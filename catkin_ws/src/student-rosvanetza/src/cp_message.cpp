#include "cp_message.hpp"
#include <boost/make_shared.hpp>
#include <etsi_its_msgs/CPM.h>
#include <vanetza/asn1/cpm.hpp>

namespace vanetza
{

boost::shared_ptr<etsi_its_msgs::CPM> convert_cpm(const asn1::Cpm& asn1, std::string* error_msg)
{
    auto msg = boost::make_shared<etsi_its_msgs::CPM>();

    // etsi_its_msgs/CPM header fields
    msg->header.stamp = ros::Time::now();
    msg->its_header.protocol_version = asn1->header.protocolVersion;
    msg->its_header.message_id = asn1->header.messageID;
    msg->its_header.station_id = asn1->header.stationID;
    msg->generation_delta_time = asn1->cpm.generationDeltaTime;

    const auto& params = asn1->cpm.cpmParameters;

    msg->station_type.value = params.managementContainer.stationType;
    const auto& refpos = params.managementContainer.referencePosition;
    msg->reference_position.altitude.value = refpos.altitude.altitudeValue;
    msg->reference_position.altitude.confidence = refpos.altitude.altitudeConfidence;
    msg->reference_position.latitude = refpos.latitude;
    msg->reference_position.longitude = refpos.longitude;
    msg->reference_position.position_confidence.semi_major_confidence = refpos.positionConfidenceEllipse.semiMajorConfidence;
    msg->reference_position.position_confidence.semi_minor_confidence = refpos.positionConfidenceEllipse.semiMinorConfidence;
    msg->reference_position.position_confidence.semi_major_orientation = refpos.positionConfidenceEllipse.semiMajorOrientation;


    if (params.stationDataContainer && params.stationDataContainer->present == StationDataContainer_PR_originatingVehicleContainer)
    {
        const auto& originVehCont= params.stationDataContainer->choice.originatingVehicleContainer;

        msg->originatingVehicleContainer.heading.value = originVehCont.heading.headingValue;
        msg->originatingVehicleContainer.heading.confidence = originVehCont.heading.headingConfidence;
        msg->originatingVehicleContainer.speed.value = originVehCont.speed.speedValue;
        msg->originatingVehicleContainer.speed.confidence = originVehCont.speed.speedConfidence;

        msg->originatingVehicleContainer.drive_direction.value = originVehCont.driveDirection;

        if(originVehCont.vehicleLength) {
            msg->originatingVehicleContainer.vehicle_length.value = originVehCont.vehicleLength->vehicleLengthValue;
            msg->originatingVehicleContainer.vehicle_length.confidence_indication = originVehCont.vehicleLength->vehicleLengthConfidenceIndication;
        }

        if(originVehCont.vehicleWidth)
            msg->originatingVehicleContainer.vehicle_width.value = *(originVehCont.vehicleWidth);

        if(originVehCont.vehicleHeight != 0) {
            //std::cout << "Veh Height " << *(originVehCont.vehicleHeight) << std::endl;
        }

        if(originVehCont.longitudinalAcceleration) {
            msg->originatingVehicleContainer.longitudinal_acceleration.value = originVehCont.longitudinalAcceleration->longitudinalAccelerationValue;
            msg->originatingVehicleContainer.longitudinal_acceleration.confidence = originVehCont.longitudinalAcceleration->longitudinalAccelerationConfidence;
        }

        if(originVehCont.yawRate){
            msg->originatingVehicleContainer.yaw_rate.value = originVehCont.yawRate->yawRateValue;
            msg->originatingVehicleContainer.yaw_rate.confidence = originVehCont.yawRate->yawRateConfidence;
        }
    }
    else
    {
        if (error_msg) *error_msg = "missing OriginatingVehicleContainer";
        return nullptr;
    }


    /* Add the code for sensor containers */
    msg->has_sensor_information_container = false;
    const auto& sensorInformationContainer = params.sensorInformationContainer;

    if(sensorInformationContainer != nullptr && sensorInformationContainer->list.count > 0) {
        msg->sensorInformationContainer.sensorsInformation = std::vector<etsi_its_msgs::SensorInformation>();
        msg->has_sensor_information_container = true;

        for (int i = 0; i < sensorInformationContainer->list.count; i++) {
            const auto& sensorsInformation = sensorInformationContainer->list.array[i];
            etsi_its_msgs::SensorInformation ros_sensorInfo;

            ros_sensorInfo.sensorID = sensorsInformation->sensorID;
            ros_sensorInfo.type = sensorsInformation->type;
            msg->sensorInformationContainer.sensorsInformation.push_back(ros_sensorInfo);

        }
    }

    const auto& objectsContainer = params.perceivedObjectContainer;

    msg->has_list_of_perceived_object = false;

    if(objectsContainer != nullptr && objectsContainer->list.count > 0){
        msg->has_list_of_perceived_object = true;
        msg->listOfPerceivedObjects.perceivedObjectContainer = std::vector<etsi_its_msgs::PerceivedObject>();

        for(int i = 0 ; i < objectsContainer->list.count ; i++){
            const auto& objCont = objectsContainer->list.array[i];

            etsi_its_msgs::PerceivedObject rosObj;

            rosObj.objectID = objCont->objectID;
            rosObj.timeOfMeasurement = objCont->timeOfMeasurement;
            rosObj.objectConfidence = objCont->objectConfidence;

            if(objCont->objectAge)
                rosObj.objectAge = *(objCont->objectAge);

            //rosObj.objectConfidence = objCont->objectConfidence;
            rosObj.xDistance.value = objCont->xDistance.value;
            rosObj.xDistance.confidence = objCont->xDistance.confidence;
            rosObj.yDistance.value = objCont->yDistance.value;
            rosObj.yDistance.confidence = objCont->yDistance.confidence;

            if(objCont->xAcceleration){
                rosObj.xAcceleration.value = objCont->xAcceleration->longitudinalAccelerationValue;
                rosObj.xAcceleration.confidence = objCont->xAcceleration->longitudinalAccelerationConfidence;
            }

            if(objCont->planarObjectDimension1){
                rosObj.planarObjectDimension1.value = objCont->planarObjectDimension1->value;
                rosObj.planarObjectDimension1.confidence = objCont->planarObjectDimension1->confidence;
            }

            if(objCont->planarObjectDimension2){
                rosObj.planarObjectDimension2.value = objCont->planarObjectDimension2->value;
                rosObj.planarObjectDimension2.confidence = objCont->planarObjectDimension2->confidence;
            }

            //rosObj.objectRefPoint.value = objCont->objectRefPoint;

            if(objCont->dynamicStatus){
                rosObj.dynamicStatus.value = *(objCont->dynamicStatus);
            }

            if(objCont->classification != nullptr && objCont->classification->list.count > 0){

                if(objCont->classification->list.array[0]->Class.present == ObjectClass__class_PR_person){
                    PersonSubclass_t &personSubclass = objCont->classification->list.array[0]->Class.choice.person;

                    rosObj.classification.value = etsi_its_msgs::StationType::PEDESTRIAN;

                } else {
                    VehicleSubclass_t &vehicleSubclass = objCont->classification->list.array[0]->Class.choice.vehicle;
                        switch (vehicleSubclass.type) {

                            case VehicleSubclassType_moped:
                                rosObj.classification.value = etsi_its_msgs::StationType::MOPED;
                                break;
                            case VehicleSubclassType_motorcycle:
                                rosObj.classification.value = etsi_its_msgs::StationType::MOTORCYCLE;
                                break;
                            case VehicleSubclassType_passengerCar:
                                rosObj.classification.value = etsi_its_msgs::StationType::PASSENGER_CAR;
                                break;
                            case VehicleSubclassType_bus:
                                rosObj.classification.value = etsi_its_msgs::StationType::BUS;
                                break;
                            case VehicleSubclassType_lightTruck:
                                rosObj.classification.value = etsi_its_msgs::StationType::LIGHT_TRUCK;
                                break;
                            case VehicleSubclassType_heavyTruck:
                                rosObj.classification.value = etsi_its_msgs::StationType::HEAVY_TRUCK;
                                break;
                            case VehicleSubclassType_trailer:
                                rosObj.classification.value = etsi_its_msgs::StationType::TRAILER;
                                break;
                            case VehicleSubclassType_specialVehicles:
                                rosObj.classification.value = etsi_its_msgs::StationType::SPECIAL_VEHICLE;
                                break;
                            case VehicleSubclassType_tram:
                                rosObj.classification.value = etsi_its_msgs::StationType::TRAM;
                                break;
                            default:
                                rosObj.classification.value= etsi_its_msgs::StationType::UNKNOWN;
                        }

                }

            }

            msg->listOfPerceivedObjects.perceivedObjectContainer.push_back(rosObj);
        }
    }


    msg->numberOfPerceivedObjects = params.numberOfPerceivedObjects;


    return msg;
}


asn1::Cpm convertCpm(etsi_its_msgs::CPMConstPtr ptr)
{
    asn1::Cpm msg;

    ItsPduHeader_t& header = msg->header;
    header.protocolVersion = ptr->its_header.protocol_version;
    header.messageID = ptr->its_header.message_id;
    header.stationID = ptr->its_header.station_id;

    CollectivePerceptionMessage_t& cpm = (*msg).cpm;
    cpm.generationDeltaTime = ptr->generation_delta_time;


    //Management container
    CpmManagementContainer_t& managementContainer = cpm.cpmParameters.managementContainer;
    managementContainer.stationType = ptr->station_type.value;
    managementContainer.referencePosition.altitude.altitudeValue = ptr->reference_position.altitude.value;
    managementContainer.referencePosition.altitude.altitudeConfidence = ptr->reference_position.altitude.confidence;
    managementContainer.referencePosition.longitude = ptr->reference_position.longitude;
    managementContainer.referencePosition.latitude = ptr->reference_position.latitude;
    managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
            ptr->reference_position.position_confidence.semi_minor_confidence;
    managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
            ptr->reference_position.position_confidence.semi_major_confidence;
    managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation =
            ptr->reference_position.position_confidence.semi_major_orientation;

    //Station data container choice
    StationDataContainer_t*& stationDataContainer = cpm.cpmParameters.stationDataContainer;
    stationDataContainer = vanetza::asn1::allocate<StationDataContainer_t>();
    stationDataContainer->present = StationDataContainer_PR_originatingVehicleContainer;

    //Consider only vehicles for the moment
    OriginatingVehicleContainer_t& originVehCont = stationDataContainer->choice.originatingVehicleContainer;
    originVehCont.heading.headingValue = ptr->originatingVehicleContainer.heading.value;
    originVehCont.heading.headingConfidence = ptr->originatingVehicleContainer.heading.confidence;
    originVehCont.speed.speedValue = ptr->originatingVehicleContainer.speed.confidence;
    originVehCont.speed.speedConfidence = ptr->originatingVehicleContainer.speed.confidence;
    //originVehCont.driveDirection = ptr->originatingVehicleContainer.drive_direction.value;

    LongitudinalAcceleration_t*& longitudinalAcceleration = originVehCont.longitudinalAcceleration;
    longitudinalAcceleration = vanetza::asn1::allocate<LongitudinalAcceleration_t>();
    longitudinalAcceleration->longitudinalAccelerationValue =
            ptr->originatingVehicleContainer.longitudinal_acceleration.value;
    longitudinalAcceleration->longitudinalAccelerationConfidence =
            ptr->originatingVehicleContainer.longitudinal_acceleration.confidence;


    YawRate_t*& yawRate = originVehCont.yawRate;
    yawRate = vanetza::asn1::allocate<YawRate_t>();
    yawRate->yawRateValue = ptr->originatingVehicleContainer.yaw_rate.value;
    yawRate->yawRateConfidence = ptr->originatingVehicleContainer.yaw_rate.confidence;

    //TODO add vehicle dimension if value there
    //VehicleLength_t*& vehLength = originVehCont.vehicleLength;

    /*if(ptr->has_sensor_information_container){
        SensorInformationContainer_t *&seqSensInfCont = cpm.cpmParameters.sensorInformationContainer;
        seqSensInfCont = vanetza::asn1::allocate<SensorInformationContainer_t>();

        for(auto sensorInfo : ptr->sensorInformationContainer.sensorsInformation) {

            SensorInformation_t *sensorInfoCont = vanetza::asn1::allocate<SensorInformation_t>();
            sensorInfoCont->sensorID = sensorInfo.sensorID;
            sensorInfoCont->type = sensorInfo.type;


            sensorInfoCont.details.present = SensorDetails_PR_vehicleSensor;
            VehicleSensor_t &vehicleSensor = sensorInfoCont->details.choice.vehicleSensor;

            std::pair<long, long> positionPair = artery::relativePosition(sensor->position());

            vehicleSensor.refPointId = 0;
            vehicleSensor.xSensorOffset = positionPair.first;//positionPair.first / boost::units::si::meter;
            vehicleSensor.ySensorOffset = positionPair.second;//positionPair.second / boost::units::si::meter;

            //In our case only add 1 vehicle sensor properties for each sensor
            VehicleSensorProperties_t *vehicleSensorProp = vanetza::asn1::allocate<VehicleSensorProperties_t>();

            vehicleSensorProp->range = sensor->getFieldOfView()->range.value() * Range_oneMeter;


            const double openingAngleDeg = sensor->getFieldOfView()->angle / boost::units::degree::degrees;
            const double sensorPositionDeg = artery::relativeAngle(sensor->position()) / boost::units::degree::degrees;

            //angle anti-clockwise
            vehicleSensorProp->horizontalOpeningAngleStart =
                    std::fmod(std::fmod((sensorPositionDeg - 0.5 * openingAngleDeg),
                                        (double) 360) + 360, 360) * CartesianAngleValue_oneDegree;
            vehicleSensorProp->horizontalOpeningAngleEnd =
                    std::fmod(std::fmod((sensorPositionDeg + 0.5 * openingAngleDeg),
                                        (double) 360) + 360, 360) *
                    CartesianAngleValue_oneDegree;

            int result = ASN_SEQUENCE_ADD(&vehicleSensor.vehicleSensorProperties, vehicleSensorProp);
            if (result != 0) {
                perror("asn_set_add() failed");
                exit(EXIT_FAILURE);
            }

            int result = ASN_SEQUENCE_ADD(seqSensInfCont, sensorInfoCont);
            if (result != 0) {
                perror("asn_set_add() failed");
                exit(EXIT_FAILURE);
            }
        }

    }
*/
    if(ptr->has_list_of_perceived_object) {
        PerceivedObjectContainer_t *&perceivedObjectContainers = cpm.cpmParameters.perceivedObjectContainer;
        perceivedObjectContainers = asn1::allocate<PerceivedObjectContainer_t>();

        for(auto obj : ptr->listOfPerceivedObjects.perceivedObjectContainer){
            PerceivedObject_t *objContainer = asn1::allocate<PerceivedObject_t>();

            objContainer->objectID = obj.objectID;//obj.objectID;
            ros::Duration timeOfMeasurement = ros::Time::now() - obj.timeOfDetection;
            //std::cout << timeOfMeasurement.toNSec() / 1000 << std::endl;
            objContainer->timeOfMeasurement = timeOfMeasurement.toNSec() / 1000000; //nano to milli

            //objContainer->objectConfidence = 0; //default

            objContainer->xDistance.value = obj.xDistance.value;
            objContainer->xDistance.confidence = obj.xDistance.confidence;
            objContainer->yDistance.value = obj.yDistance.value;
            objContainer->yDistance.confidence = obj.yDistance.confidence;

            objContainer->xSpeed.value = obj.xSpeed.value;
            objContainer->xSpeed.confidence = (obj.xSpeed.confidence == 0) ? 127 : obj.xSpeed.confidence;
            objContainer->ySpeed.value = obj.ySpeed.value;
            objContainer->ySpeed.confidence = (obj.ySpeed.confidence == 0) ? 127 : obj.xSpeed.confidence;

            //objContainer->objectRefPoint = 1; //Default

            //TODO change to fulfill the usual format
            ObjectClassDescription_t *&objectClassification = objContainer->classification;
            objectClassification = vanetza::asn1::allocate<ObjectClassDescription_t>();

            ObjectClass_t *objectClass = vanetza::asn1::allocate<ObjectClass_t>();
            objectClass->confidence = 100; //Default

            if(obj.classification.value == etsi_its_msgs::StationType::PEDESTRIAN){
                objectClass->Class.present = ObjectClass__class_PR_person;
                PersonSubclass_t &personSubclass = objectClass->Class.choice.person;
                personSubclass.type = PersonSubclassType_pedestrian;
                personSubclass.confidence = 100;
            } else {
                objectClass->Class.present = ObjectClass__class_PR_vehicle;
                VehicleSubclass_t &vehicleSubclass = objectClass->Class.choice.vehicle;

                switch (obj.classification.value){

                    case etsi_its_msgs::StationType::MOPED:
                        vehicleSubclass.type = VehicleSubclassType_moped;
                        break;
                    case etsi_its_msgs::StationType::MOTORCYCLE:
                        vehicleSubclass.type = VehicleSubclassType_motorcycle;
                        break;
                    case etsi_its_msgs::StationType::PASSENGER_CAR:
                        vehicleSubclass.type = VehicleSubclassType_passengerCar;
                        break;
                    case etsi_its_msgs::StationType::BUS:
                        vehicleSubclass.type = VehicleSubclassType_bus;
                        break;
                    case etsi_its_msgs::StationType::LIGHT_TRUCK:
                        vehicleSubclass.type = VehicleSubclassType_lightTruck;
                        break;
                    case etsi_its_msgs::StationType::HEAVY_TRUCK:
                        vehicleSubclass.type = VehicleSubclassType_heavyTruck;
                        break;
                    case etsi_its_msgs::StationType::TRAILER:
                        vehicleSubclass.type = VehicleSubclassType_trailer;
                        break;
                    case etsi_its_msgs::StationType::SPECIAL_VEHICLE:
                        vehicleSubclass.type = VehicleSubclassType_specialVehicles;
                        break;
                    case etsi_its_msgs::StationType::TRAM:
                        vehicleSubclass.type = VehicleSubclassType_tram;
                        break;
                    default:
                        vehicleSubclass.type = VehicleSubclassType_unknown;
                }
                vehicleSubclass.confidence = 100;
            }


            int result = ASN_SEQUENCE_ADD(objectClassification, objectClass);
            if (result != 0) {
                perror("asn_set_add() failed");
                exit(EXIT_FAILURE);
            }

            result = ASN_SEQUENCE_ADD(perceivedObjectContainers, objContainer);
            if (result != 0) {
                perror("asn_set_add() failed");
                exit(EXIT_FAILURE);
            }
        }

    }

    cpm.cpmParameters.numberOfPerceivedObjects = ptr->numberOfPerceivedObjects;

    return msg;
}

} // namespace vanetza
