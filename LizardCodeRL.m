clc,clear
if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
end

% Control table address
ADDR.TORQUE_ENABLE              = 64;          % Control table address is different in Dynamixel model
ADDR.LED                        = 65;
ADDR.GOAL_POSITION              = 116;
ADDR.PRESENT_POSITION           = 132;
ADDR.PROFILE_VELOCITY           = 112;

% Data Byte Length
LEN.LED                         = 1;
LEN.GOAL_POSITION               = 4;
LEN.PRESENT_POSITION            = 4;

% Protocol version
PROTOCOL_VERSION                = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID                          = [1 2 3 4 5 6 7];          % Put array of ID's in order
BAUDRATE                        = 57600;
DEVICENAME                      = 'COM8';       % Check which port is being used on your controller ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE                   = 1;            % Value for enabling the torque
TORQUE_DISABLE                  = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE      = 0;            % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE      = 4095;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD     = 20;           % Dynamixel moving status threshold
PROFILE_VELOCITY                = 80;           % Profile Velocity

ESC_CHARACTER                   = 'e';          % Key for escaping loop

COMM_SUCCESS                    = 0;            % Communication Success result value
COMM_TX_FAIL                    = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize groupBulkWrite Struct
groupwrite_num = groupBulkWrite(port_num, PROTOCOL_VERSION);

% Initialize Groupbulkread Structs
groupread_num = groupBulkRead(port_num, PROTOCOL_VERSION);

dxl_comm_result = COMM_TX_FAIL;                 % Communication result
dxl_addparam_result = false;                    % AddParam result
dxl_getdata_result = false;                     % GetParam result

dxl_error = 0;                                  % Dynamixel error
dxl_led_value = 0;                              % Dynamixel LED value for write
dxl_present_position = zeros(1,length(DXL_ID)); % Present position
dxl_led_value_read = 0;                         % Dynamixel moving status

% Open port
if (openPort(port_num))
    fprintf('Port Opened Successfully. \n');
else
    unloadlibrary(lib_name);
    fprintf('Failed To Open Port!\n');
    fprintf('Closing Port And Unloading Library\n');
    closePort(port_num);
    unloadlibrary(lib_name);
    input('clc to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set Successfully. \n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('clc to terminate...\n');
    return;
end

% Enable Dynamixel Torque
for i=1:numel(DXL_ID)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR.PROFILE_VELOCITY, PROFILE_VELOCITY);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR.TORQUE_ENABLE, TORQUE_ENABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    else
        fprintf('Dynamixel [ID:%03d] Torque Enabled. Velocity Set To: %d \n', DXL_ID(i),PROFILE_VELOCITY);
    end
end
%% Import RL Joint Angles
pos = importdata('LizardRL_Radians_12-11-2025.csv');

% What collumn data is saved to in CSV
frontCollumn = 2;
backCollumn = 1;
FLCollumn = 6;
FRCollumn = 7;
HLCollumn = 3;
HRCollumn = 4;
tailCollumn = 5;
%% Initial Pos
% 2048 is the default position for motors.
% +500 to the legs so it starts standing
dxl_goal_position = [2048+0, 2048+0, 2048+500, 2048+500, 2048+500, 2048+500, 2048+0]; 
for i=1:numel(DXL_ID)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR.GOAL_POSITION, typecast(int32(dxl_goal_position(i)), 'uint32'));
end

%% Position Planning
pause(0.5)
numCycles = 1;

% Clear Structure
groupBulkReadClearParam(groupread_num);
for i=1:numel(DXL_ID)
    ok = groupBulkReadAddParam(groupread_num, DXL_ID(i), ADDR.PRESENT_POSITION, LEN.PRESENT_POSITION);
    if ~ok
        fprintf('[ID:%03d] groupBulkRead addparam failed\n', DXL_ID(i));
        return;
    end
end

for n=1:numCycles % Number of cycles
    groupBulkReadClearParam(groupread_num);
    % Add parameter storage for present position value
    for i=1:numel(DXL_ID)
        dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL_ID(i), ADDR.PRESENT_POSITION, LEN.PRESENT_POSITION);
        if dxl_addparam_result ~= true
            fprintf('[ID:%03d] groupBulkRead addparam (Position) failed \n', DXL_ID(i));
            return;
        end
    end
    
    for t=1:length(pos.data)
        m1 = 2048+rad2encoder(pos.data(t,frontCollumn)); % Upper body joint
        m2 = 2048-rad2encoder(pos.data(t,backCollumn)); % Lower body joint
        m3 = 2048-rad2encoder(pos.data(t,FLCollumn)); % Left forelimb contact
        m4 = 2048-rad2encoder(pos.data(t,FRCollumn)); % Right forelimb contact
        m5 = 2048-rad2encoder(pos.data(t,HLCollumn)); % Left hindlimb contact
        m6 = 2048-rad2encoder(pos.data(t,HRCollumn)); % Right hindlimb contact
        m7 = 2048+rad2encoder(pos.data(t,tailCollumn)); % Tail joint

        
        dxl_goal_position = [m1 m2 m3 m4 m5 m6 m7];         % Goal position array
        if any(dxl_goal_position>DXL_MAXIMUM_POSITION_VALUE)  || any(dxl_goal_position<DXL_MINIMUM_POSITION_VALUE)
            disp("Goal Position Not Within Bounds")
            return
        end
    
        % Packet 1: All goal positions
        groupBulkWriteClearParam(groupwrite_num);
        for i=1:numel(DXL_ID)
            dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL_ID(i), ADDR.GOAL_POSITION, LEN.GOAL_POSITION, typecast(int32(dxl_goal_position(i)), 'uint32'), LEN.GOAL_POSITION);
            if dxl_addparam_result ~= true
              fprintf('[ID:%03d] groupBulkWrite addparam goal position failed', DXL_ID(i));
              return;
            end
        end
        groupBulkWriteTxPacket(groupwrite_num);
    
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end
        % Delay between commands
        pause(0.01)
        % Bulkread present position and moving status
        groupBulkReadTxRxPacket(groupread_num);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end
        for i=1:numel(DXL_ID)
            % Check if groupbulkread data of Dynamixel is available
            dxl_getdata_result = groupBulkReadIsAvailable(groupread_num, DXL_ID(i), ADDR.PRESENT_POSITION, LEN.PRESENT_POSITION);
            if dxl_getdata_result ~= true
                fprintf('[ID:%03d] groupBulkRead getdata failed \n', DXL_ID(i));
                return;
            end

            % Get Dynamixel present position value
            dxl_present_position(i) = groupBulkReadGetData(groupread_num, DXL_ID(i), ADDR.PRESENT_POSITION, LEN.PRESENT_POSITION);
            fprintf('[ID:%03d] Present Position : %d \n', DXL_ID(i), typecast(uint32(dxl_present_position(i)), 'int32'));

        end
    end
end
pause(5)

%% Close and Disable
for i=1:numel(DXL_ID)
    % Disable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR.TORQUE_ENABLE, TORQUE_DISABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);
disp("Successfully Unloaded and Closed")
close all;
clear;

%% Rad 2 Encoder Function
function [angle] = rad2encoder(angle)
%Converts angles in radians to encoder values
angle = angle * (180/pi) * (4096/360);
end