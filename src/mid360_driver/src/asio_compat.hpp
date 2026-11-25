#pragma once

// LOCAL PATCH M1:本机无 libasio-dev,用系统 Boost.Asio 顶替 standalone Asio,上游代码不用改
#include <boost/asio.hpp>

namespace asio {
    using namespace boost::asio;
    using error_code = boost::system::error_code;
}
