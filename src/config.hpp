#pragma once

#include <boost/asio.hpp>
#include <boost/beast.hpp>

namespace program
{
    namespace net = boost::asio;
    namespace beast = boost::beast;
    namespace ssl = boost::asio::ssl;

    namespace system = boost::system;
}