#ifndef PCD_INPUT_ITERATOR_H
#define PCD_INPUT_ITERATOR_H

#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <QDebug>
#include <QFileInfo>
#include <QSettings>
#include <QDir>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <iterator>

#include "core/base/scannertypes.h"

class PcdInputIterator : public std::iterator<std::bidirectional_iterator_tag, const Frame> {
public:
    PcdInputIterator()
        : index(-1)
        , configs(new QSettings("configs.ini", QSettings::IniFormat))
    {
    }

    //Note range is [from; to]
    PcdInputIterator(QSettings* settings_, const uint& from_, const uint& to_, const uint& step_)
        : settings(settings_)
        , configs(new QSettings("configs.ini", QSettings::IniFormat))
        , index(-1)
        , from(from_)
        , to(to_)
        , step(step_)
    {
        if (to <= from || step > to - from) {
            throw std::invalid_argument("PcdInputIterator (to <= from || step > to - from)");
        }

        if (settings == nullptr) {
            throw std::invalid_argument("PcdInputIterator settings == nullptr");
        }

        initialize_range();
        initialize_filename_patterns();

        ++(*this);
    }

    uint getUpperBound()
    {
        return to;
    }

    uint getLowerBound()
    {
        return from;
    }

    bool operator==(const PcdInputIterator& it) const
    {
        if ((range.size() != it.range.size()) && (!range.empty() && !it.range.empty())) {
            throw std::runtime_error("PcdInputIterator::operator== comparing different ranges!");
        }

        const bool equal_indexes = index == it.index;
        const bool both_end = (index == -1 && it.index == it.range.size()) || (it.index == -1 && index == range.size());

        return both_end || equal_indexes;
    }

    bool operator!=(const PcdInputIterator& it) const
    {
        return !(*this == it);
    }

    Frame operator*() const
    {
        if (index == range.size()) {
            throw std::range_error("PcdInputIterator::operator*()");
        }

        Frame frame;
        const bool success = frame.load(cloud_filename_pattern.arg(range[index]), image_filename_pattern.arg(range[index]));
        qDebug() << "Loading frame #" << range[index] << (success ? ": Success" : ": Error");

        return frame;
    }

    PcdInputIterator& operator++()
    {
        if (index < int(range.size())) {
            ++index;
            return *this;
        }

        throw std::out_of_range("PcdInputIterator::operator++");
    }

    PcdInputIterator operator++(int)
    {
        PcdInputIterator tmp(*this);
        ++tmp;
        return tmp;
    }

    PcdInputIterator& operator--()
    {
        if (index > 0) {
            --index;
            return *this;
        }

        throw std::out_of_range("PcdInputIterator::operator--");
    }

    PcdInputIterator operator--(int)
    {
        PcdInputIterator tmp(*this);
        ++tmp;
        return tmp;
    }

private:
    QSettings* settings;
    QSettings* configs;
    QString image_filename_pattern;
    QString cloud_filename_pattern;

    std::vector<uint> range;
    int index;

    uint from;
    uint to;
    uint step;

    void initialize_filename_patterns()
    {
        cloud_filename_pattern = QFileInfo(settings->fileName()).absolutePath() + "/"
            + settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "/"
            + configs->value("READING_PATTERNS_SETTINGS/POINT_CLOUD_NAME").toString();

        image_filename_pattern = QFileInfo(settings->fileName()).absolutePath() + "/"
            + settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "/"
            + configs->value("READING_PATTERNS_SETTINGS/POINT_CLOUD_IMAGE_NAME").toString();
    }

    void get_all(
        const boost::filesystem::path& root,
        const string& ext,
        vector<boost::filesystem::path>& ret)
    {
        if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
            return;

        for (auto& p : boost::filesystem::directory_iterator(root))
        {
            const bool is_regular_file = boost::filesystem::is_regular_file(p.path());
            const auto str = p.path().string();
            const bool same_extension = QFileInfo(str.c_str()).completeSuffix().toStdString() == ext;
            if (is_regular_file && same_extension)
            {
                ret.push_back(p.path().filename());
            }
        }
    }

    void initialize_range()
    {        
        std::vector<boost::filesystem::path> pcd_pathes, img_pathes;
        boost::filesystem::path data_folder_path = QDir::toNativeSeparators(QFileInfo(settings->fileName()).absoluteDir().path()).toStdString();
        const std::string pcd_folder_path = settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString().toStdString();
        data_folder_path /= pcd_folder_path;

        get_all(data_folder_path, ".pcd", pcd_pathes);
        get_all(data_folder_path, ".bmp", img_pathes);

        std::vector<uint> pcd_indexes, img_indexes;
        boost::regex exp("(\\d+)");
        for (uint i = 0; i < pcd_pathes.size(); ++i) {
            boost::smatch match;
            std::string filename = pcd_pathes[i].stem().string();
            if (boost::regex_search(filename, match, exp)) {
                std::string str(match[1]);
                pcd_indexes.push_back(std::atoi(str.c_str()));
            }
        }
        for (uint i = 0; i < img_pathes.size(); ++i) {
            boost::smatch match;
            std::string filename = img_pathes[i].stem().string();
            if (boost::regex_search(filename, match, exp)) {
                std::string str(match[1]);
                img_indexes.push_back(std::atoi(str.c_str()));
            }
        }

        std::vector<uint> tmp_range;
        std::sort(pcd_indexes.begin(), pcd_indexes.end());
        std::sort(img_indexes.begin(), img_indexes.end());
        for (auto it = pcd_indexes.begin(); it != pcd_indexes.end(); ++it) {
            if (std::find(img_indexes.begin(), img_indexes.end(), *it) != img_indexes.end()) {
                tmp_range.push_back(*it);
            }
        }

        if (!tmp_range.empty()) {
            if (from < tmp_range.front()) {
                from = tmp_range.front();
            }
            if (to > tmp_range.back()) {
                to = from;
                while (to + step <= tmp_range.back()) {
                    to += step;
                }
            }

            auto from_it = std::find(tmp_range.begin(), tmp_range.end(), from);
            auto to_it = std::find(tmp_range.begin(), tmp_range.end(), to);

            for (; int(from_it - tmp_range.begin()) + step <= to_it - tmp_range.begin(); from_it += step) {
                range.push_back(*from_it);
            }
            range.push_back(*from_it);

            from = range.front();
            to = range.back();
        }
    }
};

#endif // PCD_INPUT_ITERATOR_H
