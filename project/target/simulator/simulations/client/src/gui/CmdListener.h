#pragma once
#include <atomic>
#include <string>
#include <map>
#include <thread>
#include <iostream>

class CmdListener {
private:
	std::thread thread;
	std::atomic<bool> running;
	std::map<std::string, std::function<void(std::string,std::string)>> commands;
    std::map<std::string, std::string> help;

    void printHelp(std::string arg0,std::string arg1);

public:
	CmdListener();
	CmdListener(const CmdListener &);
	~CmdListener() {};
	CmdListener(std::map<std::string,std::function<void(std::string,std::string)>> &commands, std::map<std::string,std::string> &help);
    void listen();
    void addCommand(std::string command,std::string help,const std::function<void(std::string,std::string)> &fcn);
    void printHelp();

};
