# CHANGE HERE ACCORDING TO YOUR ENVIRONNMENT
# Configuration environnement CERTI
export CERTI_HOME=~/Dev/CERTI/install/
source $CERTI_HOME/share/scripts/myCERTI_env.sh 

sleep 0.1
gnome-terminal -e rtig 

# On laisse au systeme le temps d'ouvrir des shells
SLEEP_TIME=1
sleep $SLEEP_TIME
gnome-terminal -e "./EngineFederateHla1516e 0.1" &
sleep $SLEEP_TIME 
# "<IsExternalFcs> <BridgeIpAddress> <BridgeUdpPort> <FcsIpAddress> <FcsUdpPort>"
# EXTERNAL
# On the board
# >> cd /root/SMARTIES/install/bin
# >> ./fcs_entity 10.10.9.1 1 10.10.9.2 2
gnome-terminal -e "./EfcsBridgeFederateHla1516e 0 10.10.9.1 6001 10.10.9.2 6002" &
# NOT EXTERNAL
# gnome-terminal -e "./EfcsBridgeFederateHla1516e 0 10.10.9.1 1 10.10.9.2 2" &
sleep $SLEEP_TIME
gnome-terminal -e "./ManagerFederateHla1516e" &
sleep $SLEEP_TIME
gnome-terminal -e "./VisualizationFederateHla1516e" &
sleep $SLEEP_TIME
gnome-terminal -e "./CockpitFederateHla1516e" &
sleep $SLEEP_TIME
gnome-terminal -e "./FlightDynamicsFederateHla1516e 0.1" &
sleep $SLEEP_TIME

