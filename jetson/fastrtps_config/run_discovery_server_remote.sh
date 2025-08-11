#!/bin/bash

# Initialize variables
SCRIPT_DIRECTORY=""
DEFAULT_REMOTE_GUID=""
DEFAULT_REMOTE_IP=""
DEFAULT_REMOTE_PORT=""
NUM_SERVERS=""
additional_guid=()
additional_ip=()
additional_port=()
all_remote_guids=()
SERVER_ID=""
SERVER_ID_HEX=""
SERVER_GUID=""
HOST_IP=""
FIXED_PORT=""
DISCOVERY_PORT=""
TEMPLATE_FILE=""

SCRIPT_DIRECTORY="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)"

# Warning: This script must be sourced.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "Warning: This script must be sourced. Use '. ./discovery_server.sh' or 'source discovery_server.sh' instead of './discovery_server.sh'."
    exit 1
fi

# Set Fast-RTPS implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Set ROS2 CLI to use Discovery SUPER_CLIENT configuration
echo "Setting ROS2 CLI to use Discovery SUPER_CLIENT configuration."
export FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIRECTORY}/discovery_super_client.xml"
ros2 daemon stop
ros2 daemon start
echo ""

# Default connection target information
DEFAULT_REMOTE_GUID="44.53.00.5f.45.50.52.4f.53.49.4d.41"
DEFAULT_REMOTE_IP="127.0.0.1"
DEFAULT_REMOTE_PORT=11811
echo "Default connection target:"
echo "  GUID: ${DEFAULT_REMOTE_GUID}"
echo "  IP  : ${DEFAULT_REMOTE_IP}"
echo "  Port: ${DEFAULT_REMOTE_PORT}"
echo ""

# Prompt for additional remote servers
echo "Enter the number of additional remote servers (>= 0):"
read NUM_SERVERS
if ! [[ "$NUM_SERVERS" =~ ^[0-9]+$ ]]; then
    echo "Invalid number entered. Exiting."
    exit 1
fi

# Initialize duplicate check array with default GUID (only in this run)
all_remote_guids=("${DEFAULT_REMOTE_GUID}")

for (( i=1; i<=NUM_SERVERS; i++ )); do
    echo "-----------------------------------------"
    echo "Configuring remote server $i:"
    echo "Note: The GUID must not duplicate any existing connection target GUID."
    echo "Example: 44.53.A1.5f.45.50.52.4f.53.49.4d.42"
    echo "Enter GUID for remote server $i:"
    read REMOTE_GUID

    duplicate=false
    for guid in "${all_remote_guids[@]}"; do
        if [ "$REMOTE_GUID" == "$guid" ]; then
            duplicate=true
            break
        fi
    done
    if [ "$duplicate" = true ]; then
         echo "Error: The GUID '$REMOTE_GUID' is already used. Please enter a unique GUID."
         i=$((i-1))
         continue
    fi
    additional_guid+=("$REMOTE_GUID")
    all_remote_guids+=("$REMOTE_GUID")

    echo "Enter IP address for remote server $i (e.g., 192.168.1.10):"
    read REMOTE_ADDRESS
    additional_ip+=("$REMOTE_ADDRESS")

    echo "Enter port for remote server $i (e.g., 11811):"
    read REMOTE_PORT
    additional_port+=("$REMOTE_PORT")
done
echo ""

# Prompt for the server ID after collecting all connection targets.
while true; do
    echo "Enter the server ID to launch. Its corresponding GUID must not duplicate any connection target GUID. (e.g., 2):"
    read SERVER_ID
    if ! [[ "$SERVER_ID" =~ ^[0-9]+$ ]]; then
        echo "Invalid server ID. Please enter a numeric value."
        continue
    fi
    SERVER_ID_HEX=$(printf "%02X" "${SERVER_ID}")
    # Generate server GUID in the form: 44.53.<SERVER_ID_HEX>.5f.45.50.52.4f.53.49.4d.41
    SERVER_GUID="44.53.${SERVER_ID_HEX}.5f.45.50.52.4f.53.49.4d.41"
    duplicate=false
    for guid in "${all_remote_guids[@]}"; do
        if [ "$SERVER_GUID" == "$guid" ]; then
            duplicate=true
            break
        fi
    done
    if [ "$duplicate" = true ]; then
         echo "Error: The GUID corresponding to the server ID ($SERVER_GUID) duplicates an existing connection target GUID. Please choose another server ID."
         continue
    fi
    break
done

echo "Selected server ID: ${SERVER_ID} corresponds to GUID: ${SERVER_GUID}"

# Detect host IP address
HOST_IP=$(hostname -I | awk '{print $1}')
echo "Detected host IP address: ${HOST_IP}"

# Function: Check if a given UDP port is in use using lsof.
is_port_in_use() {
    local port=$1
    if lsof -iUDP:"${port}" >/dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

FIXED_PORT=${DEFAULT_REMOTE_PORT}
DISCOVERY_PORT=11812

# Check if DISCOVERY_PORT is in use; if so, increment the port number.
while is_port_in_use "${DISCOVERY_PORT}"; do
    echo "Port ${DISCOVERY_PORT} is in use. Incrementing port number."
    DISCOVERY_PORT=$((DISCOVERY_PORT + 1))
done

echo "Using fixed RemoteServer port: ${FIXED_PORT}"
echo "Using Discovery Server port: ${DISCOVERY_PORT}"
echo "Participant GUID corresponding to server ID: ${SERVER_GUID}"
echo ""

# XML configuration file location in /tmp with filename containing the server ID.
TEMPLATE_FILE="/tmp/custom_server_${SERVER_ID}.xml"

# Remove existing XML file if it exists.
if [ -f "${TEMPLATE_FILE}" ]; then
    echo "Existing XML configuration file ${TEMPLATE_FILE} found. Removing it."
    rm "${TEMPLATE_FILE}"
fi

echo "Creating XML configuration file (location: ${TEMPLATE_FILE})..."
cat <<EOF > "${TEMPLATE_FILE}"
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile_server_full_example" is_default_profile="true">
        <rtps>
            <!-- GUID corresponding to the server ID -->
            <prefix>${SERVER_GUID}</prefix>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SERVER</discoveryProtocol>
                    <discoveryServersList>
                        <!-- Default RemoteServer -->
                        <RemoteServer prefix="${DEFAULT_REMOTE_GUID}">
                            <metatrafficUnicastLocatorList>
                                <locator>
                                    <udpv4>
                                        <address>${DEFAULT_REMOTE_IP}</address>
                                        <port>${FIXED_PORT}</port>
                                    </udpv4>
                                </locator>
                            </metatrafficUnicastLocatorList>
                        </RemoteServer>
EOF

# Append additional remote server configurations to the XML file.
for (( i=0; i<NUM_SERVERS; i++ )); do
    cat <<EOF >> "${TEMPLATE_FILE}"
                        <RemoteServer prefix="${additional_guid[$i]}">
                            <metatrafficUnicastLocatorList>
                                <locator>
                                    <udpv4>
                                        <address>${additional_ip[$i]}</address>
                                        <port>${additional_port[$i]}</port>
                                    </udpv4>
                                </locator>
                            </metatrafficUnicastLocatorList>
                        </RemoteServer>
EOF
done

# Close XML tags.
cat <<EOF >> "${TEMPLATE_FILE}"
                    </discoveryServersList>
                </discovery_config>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>${HOST_IP}</address>
                            <port>${DISCOVERY_PORT}</port>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
            </builtin>
        </rtps>
    </participant>

    <data_writer profile_name="default publisher profile" is_default_profile="true">
        <qos>
            <data_sharing>
                <kind>AUTOMATIC</kind>
            </data_sharing>
        </qos>
        <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </data_writer>

    <data_reader profile_name="default subscription profile" is_default_profile="true">
        <qos>
            <data_sharing>
                <kind>AUTOMATIC</kind>
            </data_sharing>
        </qos>
        <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </data_reader>
</profiles>
EOF

echo "XML configuration file ${TEMPLATE_FILE} has been created."
echo ""

# Launch Discovery Server with the custom XML configuration.
echo "Launching Discovery Server with custom XML configuration."
fastdds discovery -i "${SERVER_ID}" -x "${TEMPLATE_FILE}"

# Cleanup: unset all temporary variables to avoid residual values in the sourced shell.
unset SCRIPT_DIRECTORY
unset DEFAULT_REMOTE_GUID
unset DEFAULT_REMOTE_IP
unset DEFAULT_REMOTE_PORT
unset NUM_SERVERS
unset additional_guid
unset additional_ip
unset additional_port
unset all_remote_guids
unset REMOTE_GUID
unset REMOTE_ADDRESS
unset REMOTE_PORT
unset SERVER_ID
unset SERVER_ID_HEX
unset SERVER_GUID
unset HOST_IP
unset FIXED_PORT
unset DISCOVERY_PORT
unset TEMPLATE_FILE
