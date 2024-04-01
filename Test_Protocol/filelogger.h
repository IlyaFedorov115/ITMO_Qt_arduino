#ifndef FILELOGGER_H
#define FILELOGGER_H
#include <QString>
#include <QTextStream>
#include <initializer_list>
#include <QDateTime>
#include <QFile>
#include <stdexcept>
namespace logging {

enum LogFileType {
    TXT,
    CSV,
    XLSX
};

template<int NumColumns>
class FileLogger {
public:

    FileLogger(const QString& directory = ".", const QString& prefix = "", LogFileType fileType = CSV)
        : directory(directory), prefix(prefix), fileType(fileType)
    {}

    void setDirectory(const QString& directory) { this->directory = directory; }
    void setPrefix(const QString& prefix) { this->prefix = prefix; }
    void setFileType(LogFileType fileType) { this->fileType = fileType; }

    void setColumnNames(const QStringList& columnNames) { this->columnNames = columnNames; }

    void startLog() {
        QString fileName = generateFileName();
        file.setFileName(fileName);

        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            throw std::runtime_error("Failed to open file for writing");
        }

        QTextStream out(&file);
        out << columnNames.join(",") << Qt::endl;
    }

    void stopLog() {
        if (file.isOpen()) {
            file.close();
        }
    }
    template <class T>
    void writeLog(std::initializer_list<T> list) {
        if (!file.isOpen()) {
            throw std::runtime_error("File is not open for writing ");
        }

        if (list.size() != NumColumns) {
            throw std::invalid_argument("Number of arguments does not match number of columns ");
        }

        QTextStream out(&file);
        /*
        for (const T& el: list) {
            out << el << ",";
        }
        out << Qt::endl;

        */

        auto it = list.begin();
           auto end = list.end();
           for (; it != end; ++it) {
               out << *it;
               if (std::next(it) != end) {
                   out << ",";
               }
           }
           out << Qt::endl;
    }

private:
    QString directory;
    QString prefix;
    LogFileType fileType;
    QFile file;
    QStringList columnNames;

    QString generateFileName() {
        QDateTime currentTime = QDateTime::currentDateTime();
        QString format;

        switch (fileType) {
            case TXT:
                format = "txt";
                break;
            case CSV:
                format = "csv";
                break;
            case XLSX:
                format = "xlsx";
                break;
        }
        return directory + "/" + prefix + "_" + currentTime.toString("yyyy_MM_dd_hh-mm-ss") + "." + format;
    }
};





}

#endif // FILELOGGER_H
