#ifndef EDGEBALANCER_HPP
#define EDGEBALANCER_HPP

#include <iterator>
#include <stdexcept>

#include "core/base/scannerbase.h"
#include "core/base/types.h"
#include "core/registration/icpregistration.h"
#include "core/registration/linearregistration.hpp"
#include "core/registration/sacregistration.h"

template <typename Metric, typename Iter>
class EdgeBalancer : public ScannerBase {
public:
    typedef Eigen::Matrix4f Matrix;

    EdgeBalancer(const Iter& begin_iterator,
        const Iter& end_iterator,
        const uint& loop_size_,
        QSettings* settings,
        QObject* parent = nullptr)
        : ScannerBase(parent, settings)
        , loop_size(loop_size_)
    {
        if (begin_iterator == end_iterator) {
            throw std::invalid_argument("EdgeBalancer::setInput begin_iterator == end_iterator");
        }
        if (loop_size <= 2) {
            throw std::invalid_argument("EdgeBalancer::setInput loop_size <= 2");
        }

        begin_it = begin_iterator;
        end_it = end_iterator;
    }

    std::vector<uint> balance()
    {
        init_edges();
        calculate_transformations();
        calculate_metrics();
        calculate_abs_deviations();
        balance_outliers();

        return getBalancedEdges();
    }

    std::vector<uint> getBalancedEdges()
    {
        std::vector<uint> result;
        std::transform(edges.begin(), edges.end(), std::back_inserter(result),
            [](const Edge& edge) { return edge.index; });
        return result;
    }

private:
    struct Edge {
        int index;
        Iter it;
        Matrix transformation;
        double metric;
        double abs_deviation;

        Edge(const int& index_, const Iter& it_)
            : index(index_)
            , it(it_)
            , transformation(Matrix::Identity())
            , metric(0)
            , abs_deviation(0)
        {
        }

        template <typename T>
        void calculate_abs_deviation(const T& average_metric)
        {
            abs_deviation = std::abs(metric - average_metric);
        }

        Edge& operator++()
        {
            ++index;
            ++it;
            return *this;
        }

        Edge& operator--()
        {
            --index;
            --it;
            return *this;
        }
    };
    typedef std::vector<Edge, Eigen::aligned_allocator<Matrix> > Edges;

    const uint loop_size;
    Iter begin_it;
    Iter end_it;
    Edges edges;
    double average_metric;
    double average_abs_deviation;

    void init_edges(const uint& begin_edge_index = 0)
    {
        uint edge_index = begin_edge_index == 0 ? 0 : begin_edge_index + 1;
        uint frame_index = edges.empty() ? 0 : edges[begin_edge_index].index;
        uint next_frame_index = begin_edge_index == 0 ? frame_index : frame_index + loop_size;
        Iter frame_it(edges.empty() ? begin_it : edges[begin_edge_index].it);

        while (frame_it != end_it) {
            const bool tail_insert_needed
                = (std::next(frame_it) == end_it) && (frame_index != next_frame_index);

            if (frame_index == next_frame_index || tail_insert_needed) {
                if (edges.size() <= edge_index) {
                    edges.push_back(Edge(frame_index, Iter(frame_it)));
                } else {
                    edges[edge_index] = Edge(frame_index, Iter(frame_it));
                }

                next_frame_index += loop_size;
                ++edge_index;
            }

            ++frame_it;
            ++frame_index;
        }

        edges.erase(edges.begin() + edge_index, edges.end());
    }

    void calculate_transformations(const uint& begin_edge_index, const uint& end_edge_index)
    {
        if (begin_edge_index >= edges.size()) {
            throw std::out_of_range("EdgeBalancer::calculate_transformations begin_edge_index >= edges.size()");
        }

        Frames edge_frames, transformed_edge_frames;
        std::transform(edges.begin() + begin_edge_index, edges.begin() + end_edge_index,
            std::back_inserter(edge_frames), [](const Edge& edge) { return *edge.it; });

        LinearRegistration<SaCRegistration> linear_sac(this, settings);
        linear_sac.setInput(edge_frames, edges[begin_edge_index].transformation);
        const Matrix4fVector sac_t = linear_sac.align(transformed_edge_frames);

        LinearRegistration<ICPRegistration> linear_icp(this, settings);
        linear_icp.setInput(transformed_edge_frames, Matrix::Identity());
        const Matrix4fVector icp_t = linear_icp.align(transformed_edge_frames);

        for (uint i = 1; i < icp_t.size(); ++i) {
            edges[i + begin_edge_index].transformation = sac_t[i] * icp_t[i];
        }
    }

    void calculate_transformations(const uint& begin_edge_index = 0)
    {
        calculate_transformations(begin_edge_index, edges.size());
    }

    void calculate_metrics(const uint& begin_edge_index, const uint& end_edge_index)
    {
        if (begin_edge_index >= edges.size() || end_edge_index > edges.size()) {
            throw std::out_of_range(
                "EdgeBalancer::calculate_metrics begin_edge_index >= edges.size() || end_edge_index > edges.size()");
        }

        for (uint i = begin_edge_index; i < end_edge_index; ++i) {
            edges[i].metric = Metric::calculate(
                edges[i].transformation,
                ((i == 0) ? Matrix::Identity() : edges[i - 1].transformation));
        }
    }

    void calculate_metrics(const uint& begin_edge_index = 0)
    {
        calculate_metrics(begin_edge_index, edges.size());
    }

    void calculate_abs_deviations()
    {
        average_metric = std::accumulate(edges.begin(), edges.end(), double(),
                             [](const double& sum, const Edge& edge) { return sum + edge.metric; })
            / edges.size();

        calculate_abs_deviations(average_metric);

        average_abs_deviation = std::accumulate(edges.begin(), edges.end(), double(),
                                    [](const double& sum, const Edge& edge) { return sum + edge.abs_deviation; })
            / edges.size();
    }

    template <typename T>
    void calculate_abs_deviations(const T& average_metric, const uint& begin_edge_index, const uint& end_edge_index)
    {
        if (begin_edge_index >= edges.size()) {
            throw std::out_of_range("EdgeBalancer::calculate_abs_deviations begin_edge_index >= edges.size()");
        }

        for (uint i = begin_edge_index; i < edges.size(); ++i) {
            edges[i].calculate_abs_deviation(average_metric);
        }
    }

    template <typename T>
    void calculate_abs_deviations(const T& average_metric, const uint& begin_edge_index = 0)
    {
        calculate_abs_deviations(average_metric, begin_edge_index, edges.size());
    }

    void balance_outliers()
    {
        for (uint i = 1; i < edges.size(); ++i) {
            if (edges[i].abs_deviation > average_abs_deviation) {
                const uint prev_index = edges[i].index;
                edges[i] = find_nearest_inlier(i);

                if (prev_index != edges[i].index) {
                    init_edges(i);
                }
            }

            if (i + 2 <= edges.size()) {
                calculate_transformations(i, i + 2);
                calculate_metrics(i, i + 2);
                calculate_abs_deviations(average_metric, i, i + 2);
            }
        }
    }

    Edge find_nearest_inlier(const uint& index)
    {
        if (edges[index].abs_deviation <= average_abs_deviation) {
            return edges[index];
        }

        Edges tmp_edges(1, edges[index]);
        Edge minus_edge(edges[index]);
        Edge plus_edge(edges[index]);

        while ((--minus_edge).index - edges[index - 1].index >= 3
            && minus_edge.abs_deviation > average_abs_deviation) {
            Frames tmp_edge_frames, tmp_transformed_edge_frames;
            tmp_edge_frames.push_back(*edges[index - 1].it);
            tmp_edge_frames.push_back(*minus_edge.it);

            LinearRegistration<SaCRegistration> linear_sac(this, settings);
            linear_sac.setInput(tmp_edge_frames, edges[index - 1].transformation);
            const Matrix4fVector sac_t = linear_sac.align(tmp_transformed_edge_frames);

            LinearRegistration<ICPRegistration> linear_icp(this, settings);
            linear_icp.setInput(tmp_transformed_edge_frames, Matrix::Identity());
            const Matrix4fVector icp_t = linear_icp.align(tmp_transformed_edge_frames);

            minus_edge.transformation = icp_t[1] * sac_t[1];
            minus_edge.metric = Metric::calculate(minus_edge.transformation, edges[index - 1].transformation);
            minus_edge.calculate_abs_deviation(average_metric);

            tmp_edges.push_back(minus_edge);
        }

        std::sort(tmp_edges.begin(), tmp_edges.end(),
            [](const Edge& a, const Edge& b) { return a.abs_deviation < b.abs_deviation; });

        while ((tmp_edges.front().abs_deviation > average_abs_deviation)
            && ((++plus_edge).index - edges[index].index <= loop_size * 2)
            && (plus_edge.it != end_it)
            && (plus_edge.abs_deviation > average_abs_deviation)) {
            Frames tmp_edge_frames, tmp_transformed_edge_frames;
            tmp_edge_frames.push_back(*edges[index - 1].it);
            tmp_edge_frames.push_back(*plus_edge.it);

            LinearRegistration<SaCRegistration> linear_sac(this, settings);
            linear_sac.setInput(tmp_edge_frames, edges[index - 1].transformation);
            const Matrix4fVector sac_t = linear_sac.align(tmp_transformed_edge_frames);

            LinearRegistration<ICPRegistration> linear_icp(this, settings);
            linear_icp.setInput(tmp_transformed_edge_frames, Matrix::Identity());
            const Matrix4fVector icp_t = linear_icp.align(tmp_transformed_edge_frames);

            plus_edge.transformation = icp_t[1] * sac_t[1];
            plus_edge.metric = Metric::calculate(plus_edge.transformation, edges[index - 1].transformation);
            plus_edge.calculate_abs_deviation(average_metric);

            tmp_edges.push_back(plus_edge);
        }

        std::sort(tmp_edges.begin(), tmp_edges.end(),
            [](const Edge& a, const Edge& b) { return a.abs_deviation < b.abs_deviation; });

        return tmp_edges.front();
    }
};

#endif //EDGEBALANCER_HPP
