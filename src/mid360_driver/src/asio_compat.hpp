#pragma once

// LOCAL PATCH M1(见 docs/PATCHES.md 的 M 项):本机没有 libasio-dev(且无 sudo),
// 上游 mid360_driver 直接依赖 standalone Asio。这里用系统自带的 Boost 1.83
// Asio 做一层极薄兼容层:namespace asio 内转发 boost::asio 的全部名字,
// 并把 error_code 明确别名到 boost::system::error_code。
// 这样 mid360_driver / mid70_driver 里的 asio::error_code、asio::awaitable、
// asio::co_spawn 等用法一行都不用改;两个 .h 只把这个头文件换进来。
#include <boost/asio.hpp>

namespace asio {
    using namespace boost::asio;
    using error_code = boost::system::error_code;
}
