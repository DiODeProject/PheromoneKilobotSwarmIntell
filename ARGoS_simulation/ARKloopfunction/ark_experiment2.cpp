/*  Authors: Mohamed S. Talamali (mstalamali1@sheffield.ac.uk) and Andreagiovanni Reina (a.reina@sheffield.ac.uk)
 *
 *  If you use this code for scientific experiment, please cite: M.S. Talamali et al. Swarm Intelligence (2019)
 *
 *  Copyright University of Sheffield, 2019
 */

#include "ark_experiment2.h"
#include <limits>
#define I2I(x,y) int(x)*MatrixSize_x+int(y)
#define I2I_OBS(x,y) int(x)*OBSMatrixSize_x+int(y)


/****************************************/
/****************************************/

CArk::CArk(): CLoopFunctions(),m_cSimulator(GetSimulator()), m_iCellsPerMetre(100),m_iCellsPerMetreOBS(100), m_ohcEmbodiedEntity(CEmbodiedEntity(NULL, "ohc_body", CVector3(0,0,1), CQuaternion())),
    m_cCommAnchor(m_ohcEmbodiedEntity.AddAnchor("comm", CVector3(0,0,0))),m_tx_flag(false), m_messaging_counter(0),m_floor_counter(0), m_update_matrix_timesteps(0), MatrixSize_x(0), MatrixSize_y(0),
    floorMatrix(0),OBSMatrixSize_x(0), OBSMatrixSize_y(0), obstacle_Matrix(0),m_showObstacleAvoidanceArea(false), maxPheroLimit(std::numeric_limits<float>::max()), evaporation_rate(0.05),
    diffusion_rate(0.02), pheromone_amount(250),m_obstacle_avoidance_range(0.05),m_Vmax(0){
}

/****************************************/
/****************************************/

CArk::~CArk(){
}

/****************************************/
/****************************************/

void CArk::Init(TConfigurationNode& t_node) {

    m_update_matrix_timesteps = 10;

    // Get the kilobots entities
    GetKilobotsEntities();

    //Create the OHC comm entities
    CreateOhcCommunicationEntities(m_KilobotsEntities.size());

    // Get the output datafile name and open it
    GetNodeAttributeOrDefault(t_node, "datafilename", m_strOutputFilename, std::string("file.out") );

    // Get the special areas in the space
    GetOptions(t_node);

    //Initialize global variables
    hasFood.resize(m_KilobotsEntities.size(),0);
    for(UInt8 i = 0; i < foodVector.size(); ++i){
        if (foodVector.at(i).id == 0) continue; // skip home
        collected[foodVector.at(i).id] = 0;
    }
    onPath.resize(foodVector.size(), 0);

    m_BringingFoodFrom.resize(m_KilobotsEntities.size(),-1);

    // Get number of pheromone matrix cells per metre
    GetNodeAttribute(t_node,"cells_per_metre",m_iCellsPerMetre);

    // Get number of obstacle avoidance matrix cells per metre
    GetNodeAttribute(t_node,"cells_per_metre_obs",m_iCellsPerMetreOBS);

    GetNodeAttribute(t_node,"obstacle_avoidance_range",m_obstacle_avoidance_range);

    GetNodeAttribute(t_node,"show_obstacle_avoidance_range",m_showObstacleAvoidanceArea);

    GetNodeAttributeOrDefault(t_node,"data_recording_start_time",m_fDataRecordingStartTimeInMinutes , ( GetSimulator().GetMaxSimulationClock()/( CPhysicsEngine::GetInverseSimulationClockTick()* 60.0)));


    m_vSpaceSize = CSimulator::GetInstance().GetSpace().GetArenaSize();

    MatrixSize_x = m_vSpaceSize.GetX()*m_iCellsPerMetre;
    MatrixSize_y = m_vSpaceSize.GetY()*m_iCellsPerMetre;

    OBSMatrixSize_x = m_vSpaceSize.GetX()*m_iCellsPerMetreOBS;
    OBSMatrixSize_y = m_vSpaceSize.GetY()*m_iCellsPerMetreOBS;

    m_vSpaceLimits = CSimulator::GetInstance().GetSpace().GetArenaLimits().GetMin();

    // Configuring the pheromone parameters
    TConfigurationNode pheroNode = GetNode(t_node,"pheromone_params");
    GetNodeAttribute(pheroNode,"evaporation_rate",evaporation_rate);
    GetNodeAttribute(pheroNode,"diffusion_rate",diffusion_rate);
    GetNodeAttribute(pheroNode,"pheromone_amount",pheromone_amount);

    // Write the parameters
    LOG << "Evap rate is " << evaporation_rate << std::endl;
    LOG << "Diff rate is " << diffusion_rate << std::endl;
    LOG << "Ph amount is " << pheromone_amount << std::endl;

    LOG << "There are " << foodVector.size()-1 << " option" << ((foodVector.size()>2)?"s":"") << endl;
    for (UInt8 i =0; i< foodVector.size(); ++i){
        if (foodVector.at(i).id==0)
            LOG << "Home is located at ";
        else
            LOG << "Option #" << foodVector[i].id << " quality: " << foodVector[i].quality << " is located at ";
        LOG << foodVector[i].position << endl;
    }

    LOG << "Data recording will start at " << m_fDataRecordingStartTimeInMinutes << " minutes"<<std::endl;

    // Create the FloorMatrix
    floorMatrix = (float*) malloc(MatrixSize_x*MatrixSize_y*sizeof(float));
    memset(floorMatrix, 0, MatrixSize_x*MatrixSize_y*sizeof(float));
    LOG << "Floor Matrix size: " << MatrixSize_x << " x " << MatrixSize_y << endl;

    //Create and Fill Obstacle Matrix
    FillObstaclesAvoidanceMatrix(GetNode(GetSimulator().GetConfigurationRoot(), "arena"));

}

/****************************************/
/****************************************/

void CArk::Reset() {
    m_messaging_counter = 0;
    m_floor_counter = 0;
    //Initialize the FloorMatrix
    memset(floorMatrix, 0, MatrixSize_x*MatrixSize_y*sizeof(float));
    memset(obstacle_Matrix, 0, MatrixSize_x*MatrixSize_y*sizeof(float));
}

/****************************************/
/****************************************/

void CArk::write_file(){
    std::ofstream m_oLogFile;
    m_oLogFile.open(m_strOutputFilename.c_str(), std::ios::out | std::ios::app);
    if(m_oLogFile.fail()) {
        THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutputFilename << "\".");
    }

    for(UInt16 i = 0; i < foodVector.size(); ++i){
        if (foodVector.at(i).id == 0) continue; // skip home
        m_oLogFile << collected[foodVector.at(i).id] << "\t";
    }


    for(UInt16 i = 0; i < foodVector.size(); ++i){
        if (foodVector.at(i).id == 0) continue; // skip home
        m_oLogFile << onPath.at(i) << "\t";
    }
    m_oLogFile << std::endl;
    m_oLogFile.close();

}

/****************************************/
/****************************************/

void CArk::Destroy() {
    //Delete floor Matrix
    free(floorMatrix);
    //Delete obstacle matrix
    free(obstacle_Matrix);
}

/****************************************/
//This function calculates the parameters of the evaporation and diffusion and actualize the matrix
/****************************************/
void CArk::updatePheromoneMatrix() {

    //Save the previous matrix in an static array
    float *aux_floorMatrix; //[MatrixSize_x][MatrixSize_y];
    aux_floorMatrix = (float*) malloc(MatrixSize_x*MatrixSize_y*sizeof(float));
    memcpy(aux_floorMatrix, floorMatrix, sizeof(float)*MatrixSize_x*MatrixSize_y );
    float diff_I, diff_J;
    float dt1=m_update_matrix_timesteps*0.1;
    float diffusionComponent = 0;

    //All except the borders
    for(int i = 1; i < MatrixSize_x-1; i++){
        for(int j = 1; j < MatrixSize_y-1; j++){
            //Calculate the vertical diffusion
            diff_I = aux_floorMatrix[I2I(i+1,j)]-2*aux_floorMatrix[I2I(i,j)]+aux_floorMatrix[I2I(i-1,j)];
            //Calculate the horizontal diffusion
            diff_J = aux_floorMatrix[I2I(i,j+1)]-2*aux_floorMatrix[I2I(i,j)]+aux_floorMatrix[I2I(i,j-1)];

            //Evaporation and diffusion equation
            diffusionComponent = diffusion_rate*(diff_I + diff_J)*dt1;
            if (diffusionComponent > maxPheroLimit) diffusionComponent = maxPheroLimit;
	    //            floorMatrix[I2I(i,j)] =  diffusionComponent - (evaporation_rate*dt1 - 1.0)*aux_floorMatrix[I2I(i,j)];
            floorMatrix[I2I(i,j)] =  aux_floorMatrix[I2I(i,j)]* Exp((Log(0.5)*evaporation_rate*dt1))+diffusionComponent;

            if(floorMatrix[I2I(i,j)] > maxPheroLimit) floorMatrix[I2I(i,j)] = maxPheroLimit;
            if(floorMatrix[I2I(i,j)] < 0.5) floorMatrix[I2I(i,j)] = 0;
        }
    }
}

/****************************************/
/****************************************/
void CArk::PreStep(){

    /* Update the time variable required for the experiment (in sec)*/
    m_fTimeInMinutes=GetSpace().GetSimulationClock()/( 60.0 * CPhysicsEngine::GetInverseSimulationClockTick() );

    //Actualize the floor every 10 steps
    m_floor_counter++;
    if(m_floor_counter >= m_update_matrix_timesteps){
        //Reduce the pheromon: Evaporation and diffusion
        updatePheromoneMatrix();
        m_floor_counter = 0;
        GetSpace().GetFloorEntity().SetChanged();
    }
    // Send messages to the kilobots using ohc every 2 clock ticks (0.2s)
    m_messaging_counter++;
    if(m_messaging_counter >= 2){
        m_messaging_counter = 0;
        CKilobotCommunicationMedium& m_cMedium(m_cSimulator.GetMedium<CKilobotCommunicationMedium>("kilocomm"));
        CKilobotCommunicationMedium::TAdjacencyMatrix& m_tCommMatrix(m_cMedium.GetCommMatrix());

        std::fill(onPath.begin(), onPath.end(), 0);

        for(unsigned int i=0;i< m_KilobotsEntities.size();i++){

            Get_message_to_send(m_KilobotsEntities[i],m_messages[i]);

            if(m_tx_flag) {
                m_ohcCommunicationEntities[i].Enable();
                m_ohcCommunicationEntities[i].SetTxStatus(CKilobotCommunicationEntity::TX_ATTEMPT);
                m_ohcCommunicationEntities[i].SetTxMessage(&m_messages[i]);

                m_tCommMatrix[ (m_KilobotsEntities[i]->GetKilobotCommunicationEntity()).GetIndex() ].insert(&m_ohcCommunicationEntities[i]);
            }
            m_tx_flag=false;
        }
    }
}
/****************************************/
/****************************************/

void CArk::PostStep(){
    if( (m_fTimeInMinutes>=m_fDataRecordingStartTimeInMinutes) && ( ((int) m_fTimeInMinutes) == m_fTimeInMinutes ) )
    {
        write_file();
    }
}

/****************************************/
/****************************************/

void CArk::CreateOhcCommunicationEntities(unsigned int number_of_comm_entities){
    m_ohcCommunicationEntities=KCVector(number_of_comm_entities, CKilobotCommunicationEntity(NULL,"kilocomm_0",9,1.0,m_cCommAnchor,m_ohcEmbodiedEntity));
    m_messages=KMVector(number_of_comm_entities);
}

/****************************************/
/****************************************/

void CArk::GetKilobotsEntities(){

    //Go through all the robots in the environment and create a vector of pointers on their entities
    CSpace::TMapPerType& kilobots_map = CSimulator::GetInstance().GetSpace().GetEntitiesByType("kilobot");

    for(CSpace::TMapPerType::iterator it = kilobots_map.begin(); it != kilobots_map.end();++it){
        m_KilobotsEntities.push_back(any_cast<CKilobotEntity*>(it->second));
    }
}

/****************************************/
/****************************************/

CVector2 CArk::GetKilobotPosition(CKilobotEntity* kilobot_entity){
    CVector3 pos3D = kilobot_entity->GetEmbodiedEntity().GetOriginAnchor().Position;
    return CVector2(pos3D.GetX(), pos3D.GetY());
}

CRadians CArk::GetKilobotOrientation(CKilobotEntity* kilobot_entity){
    CRadians cZAngle;
    CRadians cYAngle;
    CRadians cXAngle;
    CQuaternion cQuaternion = kilobot_entity->GetEmbodiedEntity().GetOriginAnchor().Orientation;
    cQuaternion.ToEulerAngles(cZAngle,cYAngle, cXAngle);
    return cZAngle;
}

/** ***************************************/
/** **************************************/

void CArk::Get_message_to_send(CKilobotEntity* kilobot_entity,message_t& message){
    //Get the kilobot position and orientation
    CVector2 kb_pos=GetKilobotPosition(kilobot_entity);
    CRadians orientation = GetKilobotOrientation(kilobot_entity);

    /****************************************/
    /****************************************/

    //Get positions and check if it is at home or food
    float track_x = kb_pos.GetX() - m_vSpaceLimits.GetX();
    float track_y = kb_pos.GetY() - m_vSpaceLimits.GetY();
    track_x = floor( track_x * m_iCellsPerMetre );
    track_y = floor( track_y * m_iCellsPerMetre );

    float OBStrack_x = kb_pos.GetX() - m_vSpaceLimits.GetX();
    float OBStrack_y = kb_pos.GetY() - m_vSpaceLimits.GetY();
    OBStrack_x = floor( OBStrack_x * m_iCellsPerMetreOBS );
    OBStrack_y = floor( OBStrack_y * m_iCellsPerMetreOBS );

    // store the home area location
    FoodClass homeArea;
    for (UInt16 i = 0; i < foodVector.size(); ++i)
    {
        if (foodVector.at(i).id == 0){ // home area has ID==0
            homeArea.position = foodVector.at(i).position;
            homeArea.radius = foodVector.at(i).radius;
            break;
        }
    }

    // check which source of food have found
    UInt16 foodSourceIdx = 0;
    bool atFood = false;

    CColor LED_color=GetKilobotLedColor(kilobot_entity);

    Real min_angle=std::numeric_limits<Real>::max();
    int min_index=-1;

    for (UInt16 i = 0; i < foodVector.size(); ++i){
        if (foodVector.at(i).id == 0){ continue; }

        //Check if it is inside a food source
        Real distanceToFood = CVector2( foodVector.at(i).position - kb_pos ).Length();
        if (foodVector.at(i).radius != 0 && distanceToFood <= foodVector.at(i).radius){
            foodSourceIdx = i;
            min_index=i;
            atFood = true;
            break;
        }

        //Look if the kilobot is foraging from a food source and which of them is
        if( LED_color==CColor::RED || LED_color==CColor::BLUE ||  LED_color==CColor::BLACK)
        {
            Real angle=Abs(NormalizedDifference( ToDegrees(kb_pos.Angle()) , ToDegrees(foodVector.at(i).position.Angle()) ).GetValue());

            if(angle<min_angle){
                min_angle=angle;
                min_index=i;
            }
        }
    }

    if(min_index!=-1){
        onPath.at(min_index) = onPath.at(min_index)+1;
    }

    /****************************************/
    /****************************************/

    CVector2 vectorToHome = homeArea.position - kb_pos;

    //Look if it is in the food and it is not going home
    bool atHome = false;
    if (atFood && hasFood[GetKilobotId(kilobot_entity)] == 0){

        hasFood[GetKilobotId(kilobot_entity)] = foodVector.at(foodSourceIdx).id;

        // Save from which source the robot is brining food in order to update max quality in the depot
        m_BringingFoodFrom[GetKilobotId(kilobot_entity)]=foodSourceIdx;
    }
    else{
        if ( vectorToHome.Length() <= homeArea.radius ){
            atHome = true;
            // Update maximum quality
            if( ( m_BringingFoodFrom[GetKilobotId(kilobot_entity)]!=-1 ) && ( foodVector.at(m_BringingFoodFrom[GetKilobotId(kilobot_entity)]).quality>m_Vmax ) ){
                m_Vmax=foodVector.at(m_BringingFoodFrom[GetKilobotId(kilobot_entity)]).quality;
                m_BringingFoodFrom[GetKilobotId(kilobot_entity)]=-1;
            }

            if (hasFood[GetKilobotId(kilobot_entity)] > 0) {
                collected[hasFood[GetKilobotId(kilobot_entity)]]++;
            }
            hasFood[GetKilobotId(kilobot_entity)] = 0;
        }
    }

    /****************************************/
    /****************************************/

    //Calculate the angle and the distance to home
    Real angleToHome = ToDegrees( vectorToHome.Rotate(-orientation).Angle() ).UnsignedNormalize().GetValue();

    /****************************************/
    /****************************************/

    //If it is going from Food to Home (led blue), print pheromone - update the floorMatrix
    if (GetKilobotLedColor(kilobot_entity) == CColor::BLUE && !atFood) {
        floorMatrix[I2I( track_x, track_y) ] += pheromone_amount;
    }

    /****************************************/
    // Obstacle detection - perform every iteration to check kilobot not close to obstacle or border
    /****************************************/


    //Calculate the orientation of the kilobots and save the obstacle values that the antenae can feel
    //If it is in the corners, don't substract the numbers
    bool atObstacle = false; /// record presence of obstacle (default 'false' represents no obstacle present)
    int radDetection = 30;
    int ObsToSend[4] = {0,0,0,0};
    CRange<CDegrees> plusMinus90( CDegrees(-90), CDegrees(90));

    //Check if obstacle or border in their surrondings
    for(int i = max(0, (int)(OBStrack_x - radDetection)) ; i < min(OBSMatrixSize_x, (int)(OBStrack_x + radDetection)); i++){
        for(int j = max(0, (int)(OBStrack_y - radDetection)) ; j < min(OBSMatrixSize_y, (int)(OBStrack_y + radDetection)); j++){
            //If it is not the kilobot, not home, and not food - analyse it
            if ( i == (int)OBStrack_x && j == (int)OBStrack_y ){ continue; }
            if(obstacle_Matrix[I2I_OBS(i,j)] > 0){ // positive value present in cell indicates an object or border is close

                //                LOG << "Yes I am near border!"<<std::endl;

                // convert obstacle_position to real world coordinates
                Real obs_x = Real(i) / m_iCellsPerMetreOBS;
                Real obs_y = Real(j) / m_iCellsPerMetreOBS;
                obs_x += m_vSpaceLimits.GetX() + 0.5/m_iCellsPerMetreOBS; // adding half-cell-size
                obs_y += m_vSpaceLimits.GetY() + 0.5/m_iCellsPerMetreOBS; // adding half-cell-size
                CVector2 obstacle_pos(obs_x, obs_y);

                CVector2 fromRobotToObstacle = (obstacle_pos - kb_pos).Rotate(-orientation);

                //Compare with the orientation, don't use if it is not detected from the antenae
                if ( plusMinus90.WithinMinBoundIncludedMaxBoundIncluded( ToDegrees(fromRobotToObstacle.Angle()).SignedNormalize() ) ){
                    if (floor(ToDegrees(fromRobotToObstacle.Angle()).UnsignedNormalize().GetValue()/45.0) == 6) ObsToSend[0] = 1; // obstacle detercted by left most antenae
                    if (floor(ToDegrees(fromRobotToObstacle.Angle()).UnsignedNormalize().GetValue()/45.0) == 7) ObsToSend[1] = 1; // obstacle detercted by front left antenae
                    if (floor(ToDegrees(fromRobotToObstacle.Angle()).UnsignedNormalize().GetValue()/45.0) == 0) ObsToSend[2] = 1; // obstacle detercted by front right antenae
                    if (floor(ToDegrees(fromRobotToObstacle.Angle()).UnsignedNormalize().GetValue()/45.0) == 1) ObsToSend[3] = 1; // obstacle detercted by right most antenae
                }
            }
        }
    }

    if(ObsToSend[0] == 1 || ObsToSend[1] == 1 || ObsToSend[2] == 1 || ObsToSend[3] == 1){
        atObstacle = true; //record that an obstacle has been detected
    }

    /****************************************/
    //Pheromone detection - only execute if no obstacle detected by previous code!
    /****************************************/
    int phToSend[4] = {0,0,0,0};
    if(!atObstacle){
        //Calculate the orientation of the kilobots and save the pheromone floor values that the antenae can feel
        //If it is in the corners, don't substract the numbers
        radDetection = 5;

        //Check if pheromone in their surrondings
        for(int i = max(0, (int)(track_x - radDetection)) ; i < min(MatrixSize_x, (int)(track_x + radDetection)); i++){
            for(int j = max(0, (int)(track_y - radDetection)) ; j < min(MatrixSize_y, (int)(track_y + radDetection)); j++){
                //If it is not the kilobot, not home, and not food - analyse it
                if ( i == (int)track_x && j == (int)track_y ){ continue; }
                if(floorMatrix[I2I(i,j)] > 0){ // pheromone present in cell

                    // convert phero_pos to real world coordinates
                    Real pt_x = Real(i) / m_iCellsPerMetre;
                    Real pt_y = Real(j) / m_iCellsPerMetre;
                    pt_x += m_vSpaceLimits.GetX() + 0.5/m_iCellsPerMetre; // adding half-cell-size
                    pt_y += m_vSpaceLimits.GetY() + 0.5/m_iCellsPerMetre; // adding half-cell-size
                    CVector2 phero_pos(pt_x, pt_y);

                    CVector2 fromRobotToPhero = (phero_pos - kb_pos).Rotate(-orientation);

                    //Compare with the orientation, don't use if it is not detected from the antenae
                    if ( plusMinus90.WithinMinBoundIncludedMaxBoundIncluded( ToDegrees(fromRobotToPhero.Angle()).SignedNormalize() ) ){
                        if (floor(ToDegrees(fromRobotToPhero.Angle()).UnsignedNormalize().GetValue()/45.0) == 6) phToSend[0] = 1; // left most antenae
                        if (floor(ToDegrees(fromRobotToPhero.Angle()).UnsignedNormalize().GetValue()/45.0) == 7) phToSend[1] = 1; // front left antenae
                        if (floor(ToDegrees(fromRobotToPhero.Angle()).UnsignedNormalize().GetValue()/45.0) == 0) phToSend[2] = 1; // front right antenae
                        if (floor(ToDegrees(fromRobotToPhero.Angle()).UnsignedNormalize().GetValue()/45.0) == 1) phToSend[3] = 1; // right most antenae
                    }

                }
            }
        }
    }

    /****************************************/
    /****************************************/

    //Construct the message to send to the kilobots
    uint8_t msg_type = 0;
    uint16_t msg_data = 0;
    message.type = NORMAL;
    message.data[0] = 0;
    message.data[1] = 0;
    message.data[2] = 0;

    //At Food - Needs the quality instead of the pherozone
    if(atFood && !atObstacle){
        //Save the quality in MSB of data0
        msg_data = foodVector.at(foodSourceIdx).quality;
        msg_data = (msg_data << 4);
        msg_data = msg_data + 512;
    }
    //At Obstacle or Border - Needs obstacle information instead of the pherozone
    else{
        if(atObstacle){
            // Let program know that values correspond to object not pheromones
            //Send the obstacle zones
            for(int i = 0; i < 4; i++){
                if (ObsToSend[i] != 0) {
                    //For every number send 1,2,4 or 8 or the sum to know where the obstacles are
                    msg_data = pow(2,i) + msg_data;
                }
            }


            //Obstacles zones in the 4 MSB
            msg_data =  (msg_data << 4) ;

            msg_data = msg_data + 768;

        }
        else{
            //Send the pheromone zones
            for(int i = 0; i < 4; i++){
                if (phToSend[i] != 0) {
                    //For every number send 1,2,4 or 8 or the sum to know where are ph
                    msg_data = pow(2,i) + msg_data;
                }
            }
            //Pherozones in the 4 MSB
            msg_data =  (msg_data << 4) ;
        }
    }


    //At Home
    if(atHome) {
        msg_data = m_Vmax;
        msg_data = (msg_data << 4);
        msg_data = msg_data + 256;
    }



    //Angle to home in the 4 LSB
    uint8_t ath = round(angleToHome/45.0);
    ath = (ath == 8)? 0 : ath;
    msg_type = ath;


    message.data[0] = message.data[0] | (GetKilobotId(kilobot_entity) >> 2);
    message.data[1] = message.data[1] | (GetKilobotId(kilobot_entity) << 6);
    message.data[1] = message.data[1] | (msg_type << 2);
    message.data[1] = message.data[1] | (msg_data >> 8);
    message.data[2] = message.data[2] | msg_data;

    // pack data with unused message at the end
    //message_t msg;
    uint16_t void_id = 1023;
    uint8_t void_type = 0;
    uint16_t void_data = 0;

    for(int i = 1; i < 3; ++i) {
        message.data[i*3] = message.data[i*3] | (void_id >> 2);
        message.data[1+i*3] = message.data[1+i*3] | (void_id << 6);
        message.data[1+i*3] = message.data[1+i*3] | (void_type << 2);
        message.data[1+i*3] = message.data[1+i*3] | (void_data >> 8);
        message.data[2+i*3] = message.data[2+i*3] | void_data;
    }

    message.crc =0;
    m_tx_flag=true;

}

/****************************************/
/****************************************/

UInt16 CArk::GetKilobotId(CKilobotEntity *kilobot_entity){
    std::string entity_id((kilobot_entity)->GetControllableEntity().GetController().GetId());
    return std::stoul(entity_id.substr(2));
}

/****************************************/
/****************************************/

CColor CArk::GetKilobotLedColor(CKilobotEntity *kilobot_entity){
    return kilobot_entity->GetLEDEquippedEntity().GetLED(0).GetColor();
}

/****************************************/
/****************************************/

CColor CArk::GetFloorColor(const CVector2 &c_position_on_plane) {

    double pt_x(c_position_on_plane.GetX()),pt_y(c_position_on_plane.GetY());
    CColor color=CColor::WHITE;
    for(UInt16 i = 0; i < foodVector.size(); ++i){
        CVector2 vectorToArea = foodVector.at(i).position - c_position_on_plane;
        if(vectorToArea.Length() <= foodVector.at(i).radius ){
            //Color depending on the argos file
            color=inttoccolor(foodVector.at(i).colour);
            //If food look the quality and put the right color
            if(foodVector.at(i).colour == 9){
                color = CColor(round(24.5*(10-foodVector.at(i).quality/1.8)),10,5);
            }
        }
    }

    // Paint the pheromone
    pt_x -= m_vSpaceLimits.GetX();
    pt_y -= m_vSpaceLimits.GetY();

    float track_x = floor(pt_x* m_iCellsPerMetre);
    float track_y = floor(pt_y* m_iCellsPerMetre);

    float OBStrack_x = floor(pt_x* m_iCellsPerMetreOBS);
    float OBStrack_y = floor(pt_y* m_iCellsPerMetreOBS);

    if(color != CColor::GREEN){
        if(floorMatrix[I2I(track_x,track_y)] >= 1){
            int pheroColourLimit = 1000;
            if(floorMatrix[I2I(track_x,track_y)] > pheroColourLimit){
                color= CColor::BLACK;
            } else {
                int normVal = floorMatrix[I2I(track_x,track_y)]*255/pheroColourLimit;
                color=CColor(230-normVal, 240, 255-normVal*0.7);
            }
        }

        if(m_showObstacleAvoidanceArea){
            if(obstacle_Matrix[I2I_OBS(OBStrack_x,OBStrack_y)]>0){
                color=CColor::BLUE;	// Paint all cells of the sreana with an obstacle blue
            }
        }
    }
    return color;
}

/****************************************/
/****************************************/

void CArk::GetOptions(TConfigurationNode& t_tree){

    TConfigurationNodeIterator itOptions;
    for(itOptions=itOptions.begin(&t_tree);
        itOptions!=itOptions.end();
        ++itOptions){
        if(itOptions->Value()=="option") {
            AddOption(*itOptions);
        }
    }
}


/****************************************/
/****************************************/

void CArk::AddOption(TConfigurationNode& t_node){
    FoodClass newFood;
    GetNodeAttribute(t_node, "id", newFood.id);
    if (newFood.id < 0) return;
    GetNodeAttribute(t_node, "quality", newFood.quality);
    GetNodeAttribute(t_node, "position", newFood.position);
    GetNodeAttribute(t_node, "radius", newFood.radius);
    GetNodeAttribute(t_node, "color", newFood.colour);
    foodVector.push_back(newFood);
}



/****************************************/
/****************************************/

void CArk::AddObstacle(TConfigurationNode& t_node){

    // Get information about the object
    CVector3 obs_origin;
    CVector3 obs_size;
    CVector3 obs_orientation;

    GetNodeAttribute(t_node,"size",obs_size);
    GetNodeAttribute(GetNode(t_node,"body"), "position", obs_origin);
    GetNodeAttribute(GetNode(t_node,"body"), "orientation", obs_orientation);

    // Find the limit of obstacle avoidance area

    CVector2 A( obs_origin.GetX()-obs_size.GetX()/2-m_obstacle_avoidance_range , obs_origin.GetY()+obs_size.GetY()/2+m_obstacle_avoidance_range )
            ,B( obs_origin.GetX()+obs_size.GetX()/2+m_obstacle_avoidance_range , obs_origin.GetY()+obs_size.GetY()/2+m_obstacle_avoidance_range )
            ,C( obs_origin.GetX()+obs_size.GetX()/2+m_obstacle_avoidance_range , obs_origin.GetY()-obs_size.GetY()/2-m_obstacle_avoidance_range )
            ,D( obs_origin.GetX()-obs_size.GetX()/2-m_obstacle_avoidance_range , obs_origin.GetY()-obs_size.GetY()/2-m_obstacle_avoidance_range );

    CRadians rot=ToRadians(CDegrees(obs_orientation.GetX()));

    A=(A-CVector2(obs_origin.GetX(),obs_origin.GetY())).Rotate(rot) +  CVector2(obs_origin.GetX(),obs_origin.GetY()) ;

    B=(B-CVector2(obs_origin.GetX(),obs_origin.GetY())).Rotate(rot) +  CVector2(obs_origin.GetX(),obs_origin.GetY()) ;

    C=(C-CVector2(obs_origin.GetX(),obs_origin.GetY())).Rotate(rot) +  CVector2(obs_origin.GetX(),obs_origin.GetY()) ;

    D=(D-CVector2(obs_origin.GetX(),obs_origin.GetY())).Rotate(rot) +  CVector2(obs_origin.GetX(),obs_origin.GetY()) ;


    A=CVector2(floor((A.GetX()- m_vSpaceLimits.GetX())*m_iCellsPerMetreOBS),floor((A.GetY()- m_vSpaceLimits.GetY())*m_iCellsPerMetreOBS));
    B=CVector2(floor((B.GetX()- m_vSpaceLimits.GetX())*m_iCellsPerMetreOBS),floor((B.GetY()- m_vSpaceLimits.GetY())*m_iCellsPerMetreOBS));
    C=CVector2(floor((C.GetX()- m_vSpaceLimits.GetX())*m_iCellsPerMetreOBS),floor((C.GetY()- m_vSpaceLimits.GetY())*m_iCellsPerMetreOBS));
    D=CVector2(floor((D.GetX()- m_vSpaceLimits.GetX())*m_iCellsPerMetreOBS),floor((D.GetY()- m_vSpaceLimits.GetY())*m_iCellsPerMetreOBS));

    CVector2 AB=A-B,AD=A-D,M,AM;

    for(int i = 0; i < OBSMatrixSize_x; i++){
        for(int j = 0; j < OBSMatrixSize_y; j++){
            M.SetX((Real) i);
            M.SetY((Real) j);
            AM=A-M;

            if( (AM.DotProduct(AB)>=0) && (AM.DotProduct(AB) <= AB.DotProduct(AB)) && (AM.DotProduct(AD)>=0) && (AM.DotProduct(AD)<=AD.DotProduct(AD)) ){
                obstacle_Matrix[I2I_OBS(i,j)] = 1;
            }
        }
    }

}

/****************************************/
/****************************************/

void CArk::FillObstaclesAvoidanceMatrix(TConfigurationNode& t_tree){

    // Create Obstacle Matrix
    obstacle_Matrix = (float*) malloc(OBSMatrixSize_x*OBSMatrixSize_y*sizeof(float));
    memset(obstacle_Matrix, 0, OBSMatrixSize_x*OBSMatrixSize_y*sizeof(float)); //set all initial values of matrix to '0'
    LOG << "Obstacle Matrix size: " << OBSMatrixSize_x << " x " << OBSMatrixSize_y << endl;

    // Fill
    TConfigurationNodeIterator itObstacles;

    for(itObstacles=itObstacles.begin(&t_tree); itObstacles!=itObstacles.end(); ++itObstacles){
        if(itObstacles->Value()=="box"){
            AddObstacle(*itObstacles);
        }
    }

}



/****************************************/
/****************************************/

CColor CArk::inttoccolor(int color_id){
    switch (color_id)
    {
    case 0:
        return CColor::BLACK;
        break;

    case 1:
        return CColor::WHITE;
        break;

    case 2:
        return CColor::RED;
        break;

    case 3:
        return CColor::GREEN;
        break;

    case 4:
        return CColor::BLUE;
        break;

    case 5:
        return CColor::MAGENTA;
        break;

    case 6:
        return CColor::CYAN;
        break;

    case 7:
        return CColor::YELLOW;
        break;

    case 8:
        return CColor::ORANGE;
        break;

    case 9:
        return CColor::BROWN;
        break;

    case 10:
        return CColor::PURPLE;
        break;

    default:
        return CColor::WHITE;
        break;
    }
}

REGISTER_LOOP_FUNCTIONS(CArk, "ark_loop_functions2")
