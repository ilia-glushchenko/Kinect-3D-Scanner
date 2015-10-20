#include "tools.h"

bool tools::copyRecursively(const QString &srcFilePath, const QString &tgtFilePath)
{
	QFileInfo srcFileInfo(srcFilePath);
	if (srcFileInfo.isDir()) {
		QDir targetDir(tgtFilePath);
		if (!targetDir.exists()) {
			if (!targetDir.mkdir(".")) return false;
		}
		targetDir.cdUp();
		QDir sourceDir(srcFilePath);
		QStringList fileNames = sourceDir.entryList(QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot | QDir::Hidden | QDir::System);
		foreach(const QString &fileName, fileNames) {
			const QString newSrcFilePath
				= srcFilePath + QLatin1Char('/') + fileName;
			const QString newTgtFilePath
				= tgtFilePath + QLatin1Char('/') + fileName;
			if (!copyRecursively(newSrcFilePath, newTgtFilePath))
				return false;
		}
	}
	else {
		if (!QFile::copy(srcFilePath, tgtFilePath))
			return false;
	}
	return true;
}