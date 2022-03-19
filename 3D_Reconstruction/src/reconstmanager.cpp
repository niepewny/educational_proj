#include "reconstmanager.h"
#include "CloudMaker.h"

ReconstManager::ReconstManager()
{
}

ReconstManager::~ReconstManager()
{
    delete cloudMaker;
}

void ReconstManager::run()
{
    cloudMaker->apply();
}
