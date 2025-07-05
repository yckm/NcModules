/**************************************************************//**
* Copyright(c) Kunming Dongdian Technology Co. LTD
 *  All rights reserved. 
 *   
 *  @file    TcpServerImpl.h
 *  @brief   This file defines the AsioAsyncTcpServerImpl class which
 *           is a concrete implementation of TcpServerImpl depends on 
 *           boost.asio.
 *   
 *  @version V2.0.0
 *  @author  xucheng
 *  @date    2022/12/22
 *   
 *  @note 更新日志，更新日志每次修改占1行，分别注明修改人、修改时间、
 *  		修改内容
 ******************************************************************/
#pragma once 

#include <thread>
#include <map>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/system/system_error.hpp>

#include "TcpServerImpl.h"

namespace Utils
{
namespace net 
{

class AsioAsyncTcpServerImpl : public TcpServerImpl
{
public:
    AsioAsyncTcpServerImpl();

    void init(int32_t port, const std::string& listenAddress = "") override;
    bool start() override;
    void stop() override;

private:
    void startAccept();
    void onNewConnection(const boost::system::error_code& ec);
    void onClientClose(const TcpConnPtr& connPtr);

    void fireConnectionCallback(const TcpConnPtr& connPtr);

    void loopProcess();

private:
    bool m_started{false};
    int32_t m_clientNumber{0};

    boost::asio::io_context m_ioContext;
    boost::asio::ip::tcp::acceptor m_acceptor;
    boost::asio::ip::tcp::socket m_newConnection;
    boost::asio::ip::tcp::endpoint m_endpoint;

    std::thread m_ioLoopThread;

    std::map<std::string, TcpConnPtr> m_connections;
};


} //namespace net 
} //namespace common