#include "CmdListener.h"
#include <iostream>
CmdListener::CmdListener() {

	commands = std::map<std::string,std::function<void(std::string,std::string)>>();
    help =std::map<std::string,std::string >();

	commands["?"] = [this](std::string arg0,std::string arg1){printHelp(arg0,arg1);};
	help["?"] = std::string("Displays this help.");


}


void CmdListener::listen() {
    std::string line;
    std::getline(std::cin, line);
    std::string command = line;
    std::string arg0;
    std::string arg1;
    int idx0 = line.find(' ');
    int idx1 = 0;
    if(idx0 > 0){
        idx1 = line.find(' ',idx0);
        command = line.substr(0,idx0);
        arg0 = line.substr(idx0);
    }
    if(idx1 > 0){
        arg0 = line.substr(idx0,idx1);
        arg1 = line.substr(idx1);
    }
    //std::cout << "Received command: " << command << "[" << arg0 << "][" << arg1 << "]" << std::endl;
    if(commands.count(command)){
        commands[command](arg0,arg1);
    }else{
        std::cout << command << ":Unknown command" <<std::endl;
    }
}

void CmdListener::printHelp(std::string arg0,std::string arg1){
    std::cout << "AirSim Shell. Available commands are: " << std::endl;

    for (const auto &cmdHelp : this->help) {
		std::cout << cmdHelp.first << " --> " << cmdHelp.second << std::endl ;
	}
}

CmdListener::CmdListener(std::map<std::string, std::function<void(std::string,std::string)>> &commands,std::map<std::string,std::string> &help) {
	this->commands = commands;
	this->help = help;

}

CmdListener::CmdListener(const CmdListener &old) {
    this->commands = old.commands;
    this->help = old.help;
}

void CmdListener::addCommand(const std::string command,std::string help,const std::function<void(std::string, std::string)> &fcn) {
    commands[command] = fcn;
    this->help[command] = help;
}

void CmdListener::printHelp() {
    this->printHelp("","");
}
