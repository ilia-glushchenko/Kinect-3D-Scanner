#ifndef PCD_INPUT_ITERATOR_H
#define PCD_INPUT_ITERATOR_H

#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED 
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <QFileInfo>
#include <QDebug>
#include <iterator>

#include "core/base/types.h"

class PcdInputIterator : public std::iterator<std::bidirectional_iterator_tag, const Frame>
{
public:
	PcdInputIterator() 
		: index(-1)
	{
	}

	PcdInputIterator(QSettings* settings_, const uint & from_, const uint & to_, const uint & step_) :
		settings(settings_),
		index(-1),
		from(from_),
		to(to_),
		step(step_)
	{
		if (to <= from || step > to - from)
		{
			throw std::invalid_argument("PcdInputIterator (to <= from || step > to - from)");
		}

		if (settings == nullptr)
		{
			throw std::invalid_argument("PcdInputIterator settings == nullptr");
		}

		initialize_range();
		initialize_filename_patterns();

		++(*this);
	}

	bool operator==(const PcdInputIterator & it) const
	{
		if ((range.size() != it.range.size()) && (!range.empty() && !it.range.empty()))
		{
			throw std::runtime_error("PcdInputIterator::operator== comparing different ranges!");
		}

		const bool equal_indexes = index == it.index;
		const bool both_end = (index == -1 && it.index == it.range.size()) || (it.index == -1 && index == range.size());
		
		return both_end || equal_indexes;		
	}

	bool operator!=(const PcdInputIterator & it) const
	{
		return !(*this == it);
	}

	Frame operator*() const
	{
		if (index == range.size())
		{
			throw std::range_error("PcdInputIterator::operator*()");
		}

		Frame frame;
		const bool success = frame.load(cloud_filename_pattern.arg(range[index]), image_filename_pattern.arg(range[index]));
		qDebug() << "Loading frame #" << range[index] << (success ? ": Success" : ": Error");

		return frame;
	}

	PcdInputIterator & operator++()
	{
		if (index < int(range.size()))
		{
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

	PcdInputIterator & operator--()
	{
		if (index > 0)
		{
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
	QString image_filename_pattern;
	QString cloud_filename_pattern;

	std::vector<uint> range;
	int index;

	uint from;
	uint to;
	uint step;

	void initialize_filename_patterns()
	{
		cloud_filename_pattern =
			QFileInfo(settings->fileName()).absolutePath() + "/" +
			settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "/" +
			settings->value("READING_PATTERNS/POINT_CLOUD_NAME").toString();

		image_filename_pattern =
			QFileInfo(settings->fileName()).absolutePath() + "/" +
			settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "/" +
			settings->value("READING_PATTERNS/POINT_CLOUD_IMAGE_NAME").toString();
	}

	void get_all(
			const boost::filesystem::path& root, 
			const string& ext, 
			vector<boost::filesystem::path>& ret
		)
	{
		if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

		boost::filesystem::recursive_directory_iterator it(root);
		boost::filesystem::recursive_directory_iterator endit;

		while (it != endit)
		{
			const std::string ext_ = it->path().extension().string();
			if (boost::filesystem::is_regular_file(*it) && ext_ == ext)
			{
				ret.push_back(it->path().filename());
			}
			++it;
		}
	}

	void initialize_range()
	{
		const QString data_folder = QFileInfo(settings->fileName()).absolutePath() + "/" +
			settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString();

		std::vector<boost::filesystem::path> pcd_pathes, img_pathes;
		get_all(data_folder.toStdString().c_str(), ".pcd", pcd_pathes);
		get_all(data_folder.toStdString().c_str(), ".bmp", img_pathes);

		std::vector<uint> pcd_indexes, img_indexes;
		boost::regex exp("(\\d+)");
		for (uint i = 0; i < pcd_pathes.size(); ++i)
		{
			boost::smatch match;
			std::string filename = pcd_pathes[i].stem().string();
			if (boost::regex_search(filename, match, exp))
			{
				std::string str(match[1]);
				pcd_indexes.push_back(std::atoi(str.c_str()));
			}
		}
		for (uint i = 0; i < img_pathes.size(); ++i)
		{
			boost::smatch match;
			std::string filename = img_pathes[i].stem().string();
			if (boost::regex_search(filename, match, exp))
			{
				std::string str(match[1]);
				img_indexes.push_back(std::atoi(str.c_str()));
			}
		}
		
		std::vector<uint> tmp_range;
		std::sort(pcd_indexes.begin(), pcd_indexes.end());
		std::sort(img_indexes.begin(), img_indexes.end());
		for (auto it = pcd_indexes.begin(); it != pcd_indexes.end(); ++it)
		{
			if (std::find(img_indexes.begin(), img_indexes.end(), *it) != img_indexes.end())
			{
				tmp_range.push_back(*it);
			}
		}


		auto from_it = std::find(tmp_range.begin(), tmp_range.end(), from);
		auto to_it = std::find(tmp_range.begin(), tmp_range.end(), to);
		for (auto it = from_it; it - tmp_range.begin() <= to_it - tmp_range.begin(); it += step)
		{
			range.push_back(*it);
		}
	
	}
};

#endif // PCD_INPUT_ITERATOR_H