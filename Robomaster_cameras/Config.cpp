#include "Config.h"

Config::Config() :
	name(name) {}


Config::Config(std::string name):
	name(name) {}

Config::~Config()
{}

std::ostream& operator<<(std::ostream& os, const Config& config)
{
	os << config.name;
	return os;
}
