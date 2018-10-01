
#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H
#define EOL_CSV "\n"
#include "Logger.h"
namespace mav{
class DataLogger
{
public:
	DataLogger(DataLogger &old);
	DataLogger(std::string logpath,std::string logfile,bool logImages);
	void append(std::vector<LogEntry> entries);
	static void log(std::string logpath, std::string logfile, std::string line);
	static void log(std::string logpath, std::string logfile, const std::vector<std::string> &line);
	static void log_buffered(std::string logpath, std::string logfile, std::string line);
protected:
	std::string logfile;
	std::string logpath;
	bool logImages;

};
}
#endif // !DATA_LOGGER_H
