#include "AsioTcpConnection.h"

#include <boost/asio/write.hpp>
#include <boost/asio/read.hpp>

#include <iostream>


namespace Utils
{
namespace net 
{

using namespace boost::asio;

AsioTcpConnection::AsioTcpConnection(boost::asio::ip::tcp::socket&& socket, const std::string& name)
    : m_socket(std::move(socket)),
      m_name(name) {}

std::string AsioTcpConnection::name() const
{
    return m_name;
}

void AsioTcpConnection::start()
{
    m_closed = false;
    setKeepaliveOption();
    startRead();

    m_isConnected = true;
}

void AsioTcpConnection::close()
{
    //std::cout << "trying to close, is closed: " << m_closed << std::endl;
    if (m_closed) return;

    m_closed = true;
    try
    {
        if (m_isConnected)
        {
            m_isConnected = false;
            m_socket.shutdown(ip::tcp::socket::shutdown_both);
        }  
        m_socket.close();
    }
    catch(boost::system::system_error e)
    {
        fireErrorCallback(e.code());
    }

    fireCloseCallback();
}

bool AsioTcpConnection::send(const uint8_t* data, int32_t length)
{
    if (!m_isConnected || length <= 0) return false;
    if (!m_socket.is_open()) return false;

    boost::system::error_code ec;
    auto bytesWritten = boost::asio::write(m_socket, boost::asio::buffer(data, length), ec);
    if (bytesWritten == length && !ec)
    {
        return true;
    }
    if (ec)
    {
        fireErrorCallback(ec);
    }

    return false;
}

void AsioTcpConnection::startRead()
{
    std::array<uint8_t, 4096> buf;
    m_socket.async_read_some(boost::asio::buffer(buf), 
        std::bind(&AsioTcpConnection::handleRead, this, std::ref(buf), std::placeholders::_1, std::placeholders::_2));
}

void AsioTcpConnection::handleRead(std::array<uint8_t, 4096>& buf, const boost::system::error_code& ec, std::size_t len)
{
    // if (ec == boost::asio::error::eof)
    // {
    //     close();
    //     return;
    // }
    // if (ec == boost::asio::error::operation_aborted)
    // {
    //     fireErrorCallback(ec);
    //     startRead();
    //     return;
    // }
    if (m_closed) return;

    if (ec)
    {
        //std::cout << "catch read error, close socket" << std::endl;
        fireErrorCallback(ec);
        close();
        return;
    }

    if (m_msgCallback != nullptr && len > 0)
    {
        m_msgCallback(shared_from_this(), buf.data(), len);
    }

    startRead();
}

void AsioTcpConnection::internalClose()
{
    if (m_closed) return;
    fireCloseCallback();
}

void AsioTcpConnection::fireCloseCallback()
{
    assert(m_closeCallback != nullptr);
    m_closeCallback(shared_from_this());
}
    
void AsioTcpConnection::fireErrorCallback(std::error_code ec)
{
    if (m_errorCallback != nullptr)
    {
        m_errorCallback(shared_from_this(), ec);
    }
}

void AsioTcpConnection::setKeepaliveOption()
{
    //m_socket.set_option(boost::asio::socket_base::keep_alive(true)); // enable SO_KEEPALIVE
    //m_socket.set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, TCP_KEEPIDLE>(10)); // secs before keepalive probes
    //m_socket.set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, TCP_KEEPINTVL>(1)); // interval between keepalive
    //m_socket.set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, TCP_KEEPCNT>(5)); // failed keepalive before declaring dead
}

} //namespace net 
} //namespace common