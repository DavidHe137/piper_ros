# Match robotics-education-lab<N> (short or FQDN) so ROS 2 peers with station launch files.
_h="$(hostname -f 2>/dev/null || hostname)"
if [[ "$_h" =~ robotics-education-lab([0-9]+) ]]; then
  export ROS_DOMAIN_ID="$((10#${BASH_REMATCH[1]}))"
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
fi
unset _h

alias p='python3 /piper_ros/scripts/p_ctrl.py'
