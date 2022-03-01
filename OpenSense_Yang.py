# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2019 Stanford University and the Authors             #
# Author(s): James Dunne                                                  #
#                                                                         #
# Licensed under the Apache License, Version 2.0 (the "License")         #
# you may not use this file except in compliance with the License.        #
# You may obtain a copy of the License at                                 #
# http://www.apache.org/licenses/LICENSE-2.0.                             #
#                                                                         #
# Unless required by applicable law or agreed to in writing, software     #
# distributed under the License is distributed on an "AS IS" BASIS,       #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         #
# implied. See the License for the specific language governing            #
# permissions and limitations under the License.                          #
# ----------------------------------------------------------------------- #

# Import OpenSim libraries
import opensim as osim
from math import pi
from rotationOrientationTable import*

# Set variables to use
modelFileName = 'Rajagopal_2015.osim';                                  # The path to an input model
orientationsFileName = 'MT_012005D6_009-001_orientations.sto';          # The path to orientation data for calibration
sensor_to_opensim_rotations = osim.Vec3(-pi/2, 0, 0);  
baseIMUName = 'pelvis_imu';                                             # The base IMU is the IMU on the base body of the model that dictates the heading (forward) direction of the model.
baseIMUHeading = 'z';                                                   # The Coordinate Axis of the base IMU that points in the heading direction. 
# The rotation of IMU data to the OpenSim world frame
visulizeCalibration = False;                                            # Boolean to Visualize the Output model
# Instantiate an IMUPlacer object
imuPlacer = osim.IMUPlacer();

# Set properties for the IMUPlacer
imuPlacer.set_model_file(modelFileName);
imuPlacer.set_orientation_file_for_calibration(orientationsFileName);
imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations);
imuPlacer.set_base_imu_label(baseIMUName);
imuPlacer.set_base_heading_axis(baseIMUHeading);

# Run the IMUPlacer
imuPlacer.run(visulizeCalibration);

# Get the model with the calibrated IMU
model = imuPlacer.getCalibratedModel();

# Print the calibrated model to file.
model.printToXML('calibrated_' + modelFileName)

# # Pseudo online test, This is to segment the offline data into different time pieces
quatTable = osim.TimeSeriesTableQuaternion(orientationsFileName)
for t in range(1000, 1200):
    quatTable = osim.TimeSeriesTableQuaternion(orientationsFileName)
    temp_quatTable = quatTable
    t_s = t*0.01
    temp_quatTable.trim(t_s-0.002, t_s+0.002)
    osim.STOFileAdapterQuaternion.write(temp_quatTable,  'orientations' + str(t) + '.sto')

# Quaternion data
quatTable = osim.TimeSeriesTableQuaternion(orientationsFileName)
orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
# Rotation to align Opensim coordinate
R = osim.Rotation(-pi/2, osim.CoordinateAxis(0))
orientationsDatacal = rotationOrientationTable(orientationsData, R)
oRefs = osim.OrientationsReference(orientationsDatacal)
mRefs = osim.MarkersReference()
cRefs = osim.SimTKArrayCoordinateReference()
visualize = 'True'
if visualize:
    model.setUseVisualizer(True)
s0 = model.initSystem()
constraint_var = 10.0
accuracy = 0.001
ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, cRefs)
ikSolver.setAccuracy = accuracy
s0.setTime(0.)
ikSolver.assemble(s0)
if visualize:
    model.getVisualizer().show(s0)
    model.getVisualizer().getSimbodyVisualizer().setShowSimTime(True)

# Pseudo online test, This is to simulate real-time application using offline data
for i in range(1000, 1200):
    sto_filename = 'orientations' + str(i) + '.sto'
    quatTable = osim.TimeSeriesTableQuaternion(sto_filename)
    orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
    orientationsDatacal = rotationOrientationTable(orientationsData, R)
    oRefs = osim.OrientationsReference(orientationsDatacal)
    ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, cRefs, constraint_var)
    times = oRefs.getTimes();
    t_s = times[0];
    ## Here are the code copied from https://github.com/pslade2/RealTimeKin, 
    # but there is no addOrientationValuesToTrack function in Opensim 4.1 and the rowVecView is a "RowVectorViewRotation",
    # can't be the input of RowVectorRotation where a "RowVector" is required. 
    #rowVecView = orientationsData.getNearestRow(t_s)
    #rowVec =  osim.RowVectorRotation(rowVecView)
    #ikSolver.addOrientationValuesToTrack(t_s, rowVec)
    #s0.setTime(t_s)
    #ikSolver.track(s0)
    s0.setTime(t_s)
    ikSolver.assemble(s0)
    if visualize:
        model.getVisualizer().show(s0)
    

