#ifndef __TOOLS_H
#define __TOOLS_H

#include <QDir>
#include <QStringList>
#include <QString>
#include <QFileInfo>

namespace tools 
{
	bool copyRecursively(const QString& srcFilePath, const QString& tgtFilePath);
	bool fileExists(const QString& filePath);
}

#endif //__TOOLS_h