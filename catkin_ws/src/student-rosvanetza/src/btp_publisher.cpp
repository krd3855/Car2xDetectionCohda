#include "btp_publisher.hpp"
#include "vanetza/BtpDataIndication.h"
#include "vanetza/GeoNetArea.h"
#include "vanetza/GeoNetDestination.h"
#include "vanetza/TrafficClass.h"
#include <vanetza/btp/header.hpp>
#include <vanetza/geonet/serialization_buffer.hpp>
#include <vanetza/net/osi_layer.hpp>
#include <algorithm>
#include <cassert>

using namespace vanetza;

GeoNetArea rosify(const geonet::Area& area)
{
    struct area_visitor : public boost::static_visitor<GeoNetArea>
    {
        area_visitor(const geonet::Area& area)
        {
            area_.latitude = area.position.latitude / units::degree;
            area_.longitude = area.position.longitude / units::degree;
            area_.angle = area.angle.value();
        }

        GeoNetArea operator()(const geonet::Rectangle& rect)
        {
            area_.type = GeoNetArea::TYPE_RECTANGLE;
            area_.distance_a = rect.a / units::si::meter;
            area_.distance_b = rect.b / units::si::meter;
            return area_;
        }

        GeoNetArea operator()(const geonet::Circle& circle)
        {
            area_.type = GeoNetArea::TYPE_CIRCLE;
            area_.distance_a = circle.r / units::si::meter;
            area_.distance_b = 0.0;
            return area_;
        }

        GeoNetArea operator()(const geonet::Ellipse& ellipse)
        {
            area_.type = GeoNetArea::TYPE_ELLIPSE;
            area_.distance_a = ellipse.a / units::si::meter;
            area_.distance_b = ellipse.b / units::si::meter;
            return area_;
        }

        GeoNetArea area_;
    };
    area_visitor visitor(area);
    return boost::apply_visitor(visitor, area.shape);
}

GeoNetDestination rosify(const geonet::DestinationVariant& destination)
{
    struct destination_visitor : public boost::static_visitor<GeoNetDestination>
    {
        GeoNetDestination operator()(const geonet::Address& addr) const
        {
            GeoNetDestination dest;
            dest.type = GeoNetDestination::TYPE_ADDRESS;
            ByteBuffer buffer;
            geonet::serialize_into_buffer(addr, buffer);
            assert(buffer.size() == dest.address.size());
            std::copy(buffer.begin(), buffer.end(), dest.address.begin());
            return dest;
        }

        GeoNetDestination operator()(const geonet::Area& area) const
        {
            GeoNetDestination dest;
            dest.type = GeoNetDestination::TYPE_AREA;
            dest.area = rosify(area);
            return dest;
        }

        GeoNetDestination operator()(const std::nullptr_t&) const
        {
            GeoNetDestination dest;
            dest.type = GeoNetDestination::TYPE_NONE;
            return dest;
        }
    };
    return boost::apply_visitor(destination_visitor(), destination);
}

TrafficClass rosify(const geonet::TrafficClass& tc)
{
    TrafficClass ros_tc;
    ros_tc.store_carry_forwarding = tc.store_carry_forward();
    ros_tc.channel_offloading = tc.channel_offload();
    ros_tc.id = tc.tc_id().raw();
    return ros_tc;
}

ros::Duration rosify(const boost::optional<geonet::Lifetime>& lt)
{
    return lt ? ros::Duration { lt->decode() / units::si::seconds } : ros::Duration();
}

BtpPublisher::BtpPublisher(ros::NodeHandle* nh) :
    publisher_(nh->advertise<vanetza::BtpDataIndication>("btp_indication", 100))
{
}

void BtpPublisher::indicate(const geonet::DataIndication& ind, std::unique_ptr<UpPacket> packet)
{
    try
    {
        btp::DataIndication btp_indication;
        switch (ind.upper_protocol)
        {
            case geonet::UpperProtocol::BTP_A:
                btp_indication = btp::DataIndication(ind, btp::parse_btp_a(*packet));
                publish(btp_indication, std::move(packet));
                break;
            case geonet::UpperProtocol::BTP_B:
                btp_indication = btp::DataIndication(ind, btp::parse_btp_b(*packet));
                publish(btp_indication, std::move(packet));
                break;
            default:
                ROS_WARN_NAMED("btp", "Dropped non-BTP packet");
                break;
        }
    }
    catch (vanetza::InputArchive::Exception& e)
    {
        ROS_ERROR_STREAM_NAMED("btp", "Parsing BTP header failed: " << e.what());
    }
}

void BtpPublisher::publish(const btp::DataIndication& ind, std::unique_ptr<UpPacket> packet)
{
    auto msg = boost::make_shared<BtpDataIndication>();
    if (ind.source_port)
    {
        msg->btp_type = BtpDataIndication::BTP_TYPE_A;
        msg->source_port = ind.source_port->host();
    }
    else if (ind.destination_port_info)
    {
        msg->btp_type = BtpDataIndication::BTP_TYPE_B;
        msg->destination_port_info = ind.destination_port_info->host();
    }
    else
    {
        ROS_WARN_NAMED("btp", "Dropped broken BTP packet");
        return;
    }

    msg->destination_port = ind.destination_port.host();
    msg->destination = rosify(ind.destination);
    msg->its_aid = ind.its_aid ? ind.its_aid.get() : 0;
    msg->permissions = ind.permissions.value_or(std::vector<uint8_t> {});
    msg->traffic_class = rosify(ind.traffic_class);
    msg->remaining_packet_lifetime = rosify(ind.remaining_packet_lifetime);

    for (const auto& layer : osi_layer_range(OsiLayer::Session, OsiLayer::Application))
    {
        auto byte_view = create_byte_view(*packet, layer);
        msg->data.insert(msg->data.end(), byte_view.begin(), byte_view.end());
    }

    ROS_DEBUG_NAMED("btp", "BTP packet received");
    publisher_.publish(msg);
}
