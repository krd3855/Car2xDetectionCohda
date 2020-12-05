#include "btp_data_service.hpp"
#include "btp_publisher.hpp"
#include "position_provider.hpp"
#include "security_id_publisher.hpp"
#include "tai_clock.hpp"
#include "vanetza_context.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/asio/io_service.hpp>
#include <nodelet/loader.h>
#include <ros/ros.h>
#include <algorithm>
#include <random>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vanetza");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ContextParams params;
    if(nh_private.getParam("multicast_ip", params.multicastIp)) {
        ROS_INFO("Vanetza joins group %s", params.multicastIp.c_str());
    } else {
        ROS_ERROR("Faild to get parameter multicast_ip");
    }

    if(nh_private.getParam("multicast_port", params.multicastPort)) {
        ROS_INFO("Vanetza transmits on port %i", params.multicastPort);
    } else {
        ROS_ERROR("Faild to get parameter multicast_port");
    }

    if(nh_private.getParam("ccu_port", params.ccuPort)) {
        ROS_INFO("CCU port: %i", params.ccuPort);
    } else {
        ROS_ERROR("Faild to get parameter ccu_port");
    }

    if(nh_private.getParam("ccu_ip", params.ccuIp)) {
        ROS_INFO("CCU IP: %s", params.ccuIp.c_str());
    } else {
        ROS_ERROR("Faild to get parameter ccu_ip");
    }

    if (nh_private.hasParam("pseudonym")) {
        std::vector<int> pseudonym;
        if (nh_private.getParam("pseudonym", pseudonym) && pseudonym.size() == 8) {
            static const int byte_mask = 0xff;
            vanetza::security::HashedId8 id;
            for (int i = 0; i < 8; ++i) {
                if(pseudonym[i] & ~byte_mask)
                    ROS_WARN("Element %i of pseudonym truncated", i);
                id[i] = pseudonym[i];
            }
            params.pseudonym = id;
        } else {
            ROS_ERROR("Pseudonym needs to be exactly 8 bytes long");
            return 1;
        }
    } else {
        // create a random pseudonym
        std::random_device rd;
        std::mt19937 rng(rd());
        std::generate(params.pseudonym.begin(), params.pseudonym.end(), [&rng]() {
            std::uniform_int_distribution<uint8_t> dist;
            return dist(rng);
        });
    }

    if (nh_private.getParam("gatekeeper", params.gatekeeper)) {
        ROS_INFO("DCC gatekeeper is %s", params.gatekeeper ? "enabled" : "disabled");
    }

    if (nh_private.getParam("soft_state_neighbours", params.softStateNeighbours)) {
        ROS_INFO("GN LocT %s soft-state neighbours", params.softStateNeighbours ? "with" : "without");
    }

    if (nh_private.getParam("fading_cbf_counters", params.fadingCbfCounters)) {
        ROS_INFO("GN CBF routing %s fading counters", params.fadingCbfCounters ? "with" : "without");
    }

    if (nh_private.getParam("publish_link_layer", params.publishLinkLayer)) {
        ROS_INFO("Publishing link layer messages is %s", params.publishLinkLayer ? "enabled" : "disabled");
    }

    if (nh_private.getParam("default_tx_power", params.defaultTxPower)) {
        ROS_INFO("Setting default transmission power to %.2f dBm", params.defaultTxPower);
    }

    using vanetza::geonet::StationType;
    std::string station_type = nh_private.param<std::string>(""
                                                             "", "UNKNOWN");
    boost::algorithm::to_lower(station_type);
    std::replace(station_type.begin(), station_type.end(), '_', ' ');
    if (station_type == "unknown") {
        params.stationType = StationType::Unknown;
    } else if (station_type == "pedestrian") {
        params.stationType = StationType::Pedestrian;
    } else if (station_type == "cyclist") {
        params.stationType = StationType::Cyclist;
    } else if (station_type == "moped") {
        params.stationType = StationType::Moped;
    } else if (station_type == "motorcycle") {
        params.stationType = StationType::Motorcycle;
    } else if (station_type == "passenger_car" || station_type == "passenger car") {
        params.stationType = StationType::Passenger_Car;
    } else if (station_type == "bus") {
        params.stationType = StationType::Bus;
    } else if (station_type == "light_truck" || station_type == "light truck") {
        params.stationType = StationType::Light_Truck;
    } else if (station_type == "heavy_truck" || station_type == "heavy truck") {
        params.stationType = StationType::Heavy_Truck;
    } else if (station_type == "trailer") {
        params.stationType = StationType::Trailer;
    } else if (station_type == "special_vehicle" || station_type == "special vehicle") {
        params.stationType = StationType::Special_Vehicle;
    } else if (station_type == "tram") {
        params.stationType = StationType::Tram;
    } else if (station_type == "rsu") {
        params.stationType = StationType::RSU;
    } else {
        ROS_WARN("Unknown station type '%s', falling back to unknown", station_type.c_str());
        params.stationType = StationType::Unknown;
    }

    if (!tai_clock::synchronized()) {
        ROS_ERROR("Host's CLOCK_TAI is not synchronized!");
    }

    boost::asio::io_service service;
    TimeTrigger trigger(service);

    auto signal_handler = [&](const boost::system::error_code& ec, int signal_number) {
        if (!ec) {
            ROS_INFO("Termination requested by %s.", signal_number == SIGINT ? "SIGINT" : "SIGTERM");
            service.stop();
            ros::shutdown();
        }
    };
    boost::asio::signal_set signals(service, SIGINT, SIGTERM);
    signals.async_wait(signal_handler);

    try {
        SecurityIdPublisher id_publisher(&nh_private);
        id_publisher.publishCommit(params.pseudonym);
        PositionProvider positioning(trigger, nh);
        BtpPublisher btp_publisher(&nh_private);
        VanetzaContext context(service, trigger, positioning, btp_publisher, params);
        context.startRouter();
        BtpDataService btp_service(&nh_private, context);

        nodelet::Loader nodelet_manager; // allow nodelets running in this process
        ros::spin();
    } catch (std::exception& ex) {
        ROS_FATAL("Exception caught: %s", ex.what());
        return 1;
    }

    return 0;
}
