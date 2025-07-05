/**************************************************************//**
* Copyright(c) Kunming Dongdian Technology Co. LTD
 *  All rights reserved. 
 *   
 *  @file    TcpServerImpl.h
 *  @brief   This file defines the AsioTcpConnection class which inheritated
 *           from TcpConnectionBase class and this implementaion depends
 *           on boost.asio.
 *   
 *  @version V2.0.0
 *  @author  xucheng
 *  @date    2022/12/22
 *   
 *  @note 更新日志，更新日志每次修改占1行，分别注明修改人、修改时间、
 *  		修改内容
 ******************************************************************/
#pragma once 

#include <boost/asio/ip/tcp.hpp>

#include "TcpConnectionBase.h"

namespace Utils
{
namespace net 
{

class AsioTcpConnection : public TcpConnectionBase, public std::enable_shared_from_this<AsioTcpConnection>
{
public:
    AsioTcpConnection(boost::asio::ip::tcp::socket&& socket, const std::string& name);

    std::string name() const override;
    void start() override;
    void close() override;

    bool send(const uint8_t* data, int32_t length) override;

private:
    void startRead();
    void handleRead(std::array<uint8_t, 4096>& buf, const boost::system::error_code& ec, std::size_t len);

    void internalClose();
    
    void fireCloseCallback();
    void fireErrorCallback(std::error_code ec);

    void setKeepaliveOption();
private:
    bool m_closed{false};
    std::string m_name;

    boost::asio::ip::tcp::socket m_socket;
};

} //namespace net 
} //namespace common