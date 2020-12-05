#ifndef VIRTUAL_CCU_HPP_FYX2IOKV
#define VIRTUAL_CCU_HPP_FYX2IOKV

#include <boost/asio.hpp>
#include <array>
#include <cstdint>
#include <iosfwd>
#include <list>
#include <string>


class LinkLayerReception;
class LinkLayerTransmission;
class Radio;
class RadioMedium;


class Radio
{
public:
    Radio(RadioMedium*, unsigned id, boost::asio::io_service&, boost::asio::ip::address, unsigned short cmd_port, unsigned short fwd_port);

    unsigned getId() const { return id_; }
    void receive(const LinkLayerReception& reception);
    void handleCommandReception(const boost::system::error_code& ec, std::size_t bytes);

private:
    RadioMedium* radio_medium_;
    unsigned id_;

    boost::asio::ip::udp::socket cmd_socket_;
    boost::asio::ip::udp::endpoint cmd_sender_;
    std::array<std::uint8_t, 2400> cmd_buffer_;
    std::string cmd_response_;
    boost::asio::ip::udp::socket fwd_socket_;
    boost::asio::ip::udp::endpoint fwd_endpoint_;
    std::string fwd_gossip_;

    bool filter_unicast_destination_ = true;
    std::string mac_address;
};

class RadioMedium
{
public:
    RadioMedium(boost::asio::io_service& io);

    void add(boost::asio::ip::address addr, unsigned short cmd_port, unsigned short fwd_port);
    void disseminate(const LinkLayerTransmission& transmission, const Radio* self);

    void record(std::ostream*);

private:
    boost::asio::io_service& io_;
    std::list<Radio> radios_;
    unsigned idx_ = 0;
    std::ostream* record_ = nullptr;
};

#endif /* VIRTUAL_CCU_HPP_FYX2IOKV */
