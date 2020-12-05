#include "btp_data_service.hpp"
#include <vanetza/btp/header_conversion.hpp>
#include <vanetza/common/byte_order.hpp>
#include <vanetza/geonet/data_request.hpp>
#include <vanetza/net/osi_layer.hpp>

vanetza::BtpData::Response rosify(const vanetza::geonet::DataConfirm& confirm)
{
    using Response = vanetza::BtpData::Response;
    using ResultCode = vanetza::geonet::DataConfirm::ResultCode;

    Response response;
    switch (confirm.result_code)
    {
        case ResultCode::Accepted:
            response.confirm = Response::CONFIRM_ACCEPTED;
            break;
        case ResultCode::Rejected_Max_SDU_Size:
            response.confirm = Response::CONFIRM_REJECTED_MAX_SDU_SIZE;
            break;
        case ResultCode::Rejected_Max_Lifetime:
            response.confirm = Response::CONFIRM_REJECTED_MAX_LIFETIME;
            break;
        case ResultCode::Rejected_Min_Repetition_Interval:
            response.confirm = Response::CONFIRM_REJECTED_MIN_REPETITION_INTERVAL;
            break;
        case ResultCode::Rejected_Unsupported_Traffic_Class:
            response.confirm = Response::CONFIRM_REJECTED_UNSUPPORTED_TRAFFIC_CLASS;
            break;
        case ResultCode::Rejected_Max_Geo_Area_Size:
            response.confirm = Response::CONFIRM_REJECTED_MAX_GEO_AREA_SIZE;
            break;
        case ResultCode::Rejected_Unspecified:
        default:
            response.confirm = Response::CONFIRM_REJECTED_UNSPECIFIED;
            break;
    }
    return response;
}

vanetza::geonet::TrafficClass derosify(const vanetza::TrafficClass& tc)
{
    return vanetza::geonet::TrafficClass { tc.store_carry_forwarding, tc.channel_offloading, tc.id };
}

bool fillDestinationArea(const vanetza::GeoNetArea& msg_area, vanetza::geonet::Area& area)
{
    using vanetza::GeoNetArea;
    using vanetza::units::degree;
    using vanetza::units::si::meter;

    if (msg_area.type == GeoNetArea::TYPE_CIRCLE)
    {
        vanetza::geonet::Circle circle;
        circle.r = msg_area.distance_a * meter;
        area.shape = circle;
    }
    else if (msg_area.type == GeoNetArea::TYPE_RECTANGLE)
    {
        vanetza::geonet::Rectangle rect;
        rect.a = msg_area.distance_a * meter;
        rect.b = msg_area.distance_b * meter;
        area.shape = rect;
    }
    else if (msg_area.type == GeoNetArea::TYPE_ELLIPSE)
    {
        vanetza::geonet::Ellipse elip;
        elip.a = msg_area.distance_a * meter;
        elip.b = msg_area.distance_b * meter;
    }
    else
    {
        return false;
    }

    area.position.latitude = msg_area.latitude * degree;
    area.position.longitude = msg_area.longitude * degree;
    area.angle = static_cast<vanetza::units::Angle>(msg_area.angle * degree);

    return true;
}

void fillDataRequest(const vanetza::BtpData::Request& msg, vanetza::geonet::DataRequest& gn)
{
    gn.upper_protocol = vanetza::geonet::UpperProtocol::BTP_B;
    gn.communication_profile = vanetza::geonet::CommunicationProfile::ITS_G5;
    gn.its_aid = msg.its_aid;
    if (!msg.lifetime.isZero())
        gn.maximum_lifetime.encode(msg.lifetime.toSec() * vanetza::units::si::seconds);
    if (msg.hop_limit > 0)
        gn.max_hop_limit = msg.hop_limit;
    gn.traffic_class = derosify(msg.traffic_class);
    if (!msg.repetition_duration.isZero())
    {
        vanetza::geonet::DataRequest::Repetition repetition;
        repetition.interval = msg.repetition_interval.toSec() * vanetza::units::si::seconds;
        repetition.maximum = msg.repetition_duration.toSec() * vanetza::units::si::seconds;
        gn.repetition = repetition;
    }
}

BtpDataService::BtpDataService(ros::NodeHandle* nh, VanetzaContext& stack) :
    stack_(stack),
    service_(nh->advertiseService("btp_request", &BtpDataService::callback, this))
{
}

bool BtpDataService::callback(vanetza::BtpData::Request& request, vanetza::BtpData::Response& response)
{
    using Request = vanetza::BtpData::Request;
    using Response = vanetza::BtpData::Response;

    if (request.btp_type != Request::BTP_TYPE_B)
    {
        ROS_ERROR_NAMED("btp", "Only BTP-B is implemented at the moment!");
        response.confirm = Response::CONFIRM_UNIMPLEMENTED;
        return true;
    }

    vanetza::btp::HeaderB btp_header;
    btp_header.destination_port = vanetza::host_cast(request.destination_port);
    btp_header.destination_port_info = vanetza::host_cast(request.destination_port_info);

    std::unique_ptr<vanetza::DownPacket> packet { new vanetza::DownPacket() };
    packet->layer(vanetza::OsiLayer::Application) = std::move(request.data);
    packet->layer(vanetza::OsiLayer::Transport) = btp_header;

    std::future<vanetza::geonet::DataConfirm> confirm_future;

    if (request.transport_type == Request::TRANSPORT_TYPE_SHB)
    {
        vanetza::geonet::ShbDataRequest shb_request(stack_.getMib());
        fillDataRequest(request, shb_request);
        confirm_future = stack_.requestTransmission(shb_request, std::move(packet));
    }
    else if (request.transport_type == Request::TRANSPORT_TYPE_GBC)
    {
        vanetza::geonet::GbcDataRequest gbc_request(stack_.getMib());
        if (request.destination.type != vanetza::GeoNetDestination::TYPE_AREA ||
            !fillDestinationArea(request.destination.area, gbc_request.destination))
        {
            response.confirm = Response::CONFIRM_REJECTED_INVALID_DESTINATION;
            return true;
        }
        fillDataRequest(request, gbc_request);
        confirm_future = stack_.requestTransmission(gbc_request, std::move(packet));
    }
    else
    {
        ROS_ERROR_NAMED("btp", "GeoNetworking transport type %d is not implemented yet!", request.transport_type);
        response.confirm = Response::CONFIRM_UNIMPLEMENTED;
        return true;
    }

    try
    {
        confirm_future.wait();
        const vanetza::geonet::DataConfirm& gn_confirm = confirm_future.get();
        response = rosify(gn_confirm);
        ROS_DEBUG_STREAM_NAMED("btp", "BTP request has been " << (gn_confirm.accepted() ? "accepted" : "rejected"));
        return true;
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_NAMED("btp", "Exception during GN-DATA.request: %s", ex.what());
        return false;
    }
}
