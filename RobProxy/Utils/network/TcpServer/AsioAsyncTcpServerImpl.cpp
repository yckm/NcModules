#include "AsioAsyncTcpServerImpl.h"

#include <iostream>

#include "AsioTcpConnection.h"


namespace Utils
{
namespace net 
{

using namespace boost::asio;

AsioAsyncTcpServerImpl::AsioAsyncTcpServerImpl()
    : m_acceptor(m_ioContext),
      m_newConnection(m_ioContext) {}

void AsioAsyncTcpServerImpl::init(int32_t port, const std::string& listenAddress)
{
    if (listenAddress != "")
    {
        m_endpoint = ip::tcp::endpoint(ip::address::from_string(listenAddress), port);
    }
    else
    {
        m_endpoint = ip::tcp::endpoint(ip::tcp::v4(), port);
    }
}

bool AsioAsyncTcpServerImpl::start()
{
    if (m_started) return true;

    try 
    {
        m_acceptor.open(m_endpoint.protocol());
        m_acceptor.set_option(ip::tcp::acceptor::reuse_address(true));
        m_acceptor.bind(m_endpoint);
        m_acceptor.listen();
    }
    catch (const boost::system::system_error& error)
    {
        std::cout << "Cannot start server: " << error.what() << std::endl;
        return false;
    }

    startAccept();
    m_started = true;
    m_ioLoopThread = std::thread(&AsioAsyncTcpServerImpl::loopProcess, this);

    return true;
}

void AsioAsyncTcpServerImpl::stop()
{
    if (!m_started) return;
    m_acceptor.close();
    m_clientNumber = 0;
    for (auto& item : m_connections)
    {
        TcpConnPtr conn(item.second);
        item.second.reset();
        conn->close();
    }
    m_connections.clear();

    m_ioContext.stop();
    m_started = false;
    if (m_ioLoopThread.joinable())
    {
        m_ioLoopThread.join();
    }

    //std::cout << "Tcp server is stopped." << std::endl;
}

void AsioAsyncTcpServerImpl::startAccept()
{
    m_acceptor.async_accept(m_newConnection, std::bind(&AsioAsyncTcpServerImpl::onNewConnection, 
        this, std::placeholders::_1));
}


void AsioAsyncTcpServerImpl::onNewConnection(const boost::system::error_code& ec)
{
    if (!m_started) return;

    if (ec)
    {
        std::cout << "Catch accept error, " << ec.value() << "： " << ec.message() << std::endl;
        return;
    }
    std::string name = "Connection" + std::to_string(++m_clientNumber) + std::string("-");
    name += m_newConnection.remote_endpoint().address().to_string();
    name += std::string(":") + std::to_string(m_newConnection.remote_endpoint().port());
    
    auto newConn = std::make_shared<AsioTcpConnection>(std::move(m_newConnection), name);
    newConn->setMessageCallback(m_msgCallback);
    newConn->setErrorCallback(m_errorCallback);
    newConn->setCloseCallback(std::bind(&AsioAsyncTcpServerImpl::onClientClose, this, std::placeholders::_1));

    newConn->start();

    fireConnectionCallback(newConn);
    m_connections[name] = newConn;

    startAccept();
}

void AsioAsyncTcpServerImpl::onClientClose(const TcpConnPtr& connPtr)
{
    auto n = m_connections.erase(connPtr->name());
    assert(n == 1);

    fireConnectionCallback(connPtr);
}

void AsioAsyncTcpServerImpl::fireConnectionCallback(const TcpConnPtr& connPtr)
{
    if (m_connCallback != nullptr)
    {
        m_connCallback(connPtr);
    }
}

void AsioAsyncTcpServerImpl::loopProcess()
{
    while (m_started)
    {
        if (m_ioContext.stopped())
        {
            m_ioContext.restart();
        }

        m_ioContext.run();
    }

    std::cout << "loop process is exited" << std::endl;
}



} //namespace net 
} //namespace common