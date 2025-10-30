FROM ros:humble-ros-base-jammy AS aptgetter

# Instala pacotes ROS 2 demo e MAVROS
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-mavros \
      git \
      bc && \
    rm -rf /var/lib/apt/lists/*

# (Opcional) Instala os scripts de geodatabase do MAVROS
RUN apt-get update && apt-get install -y \
      geographiclib-tools && \
    /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh && \
    rm -rf /var/lib/apt/lists/*

RUN cd /home && \
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    bash PX4-Autopilot/Tools/setup/ubuntu.sh && \
    apt-get update && \
    apt-get remove -y gz-harmonic && \
    apt-get install -y aptitude && \
    aptitude install -y gazebo libgazebo11 libgazebo-dev && \
    python3 -m pip install shapely && \
    rm -rf /var/lib/apt/lists/*

# Build PX4 in the Dockerfile to avoid runtime build delays
RUN cd /home/PX4-Autopilot && \
    DONT_RUN=1 make px4_sitl gazebo-classic

# install dependencies
RUN python3 -m pip install --upgrade pip setuptools wheel
COPY requirements.txt /home/mavros_navigator/
RUN python3 -m pip install --no-cache-dir -r /home/mavros_navigator/requirements.txt

# Copy project files
COPY source_code/ /home/mavros_navigator

RUN cd /home/mavros_navigator/src/harpia_msgs && \
    rm -rf build install log && \
    cd /home/mavros_navigator && \
    rm -rf build install log

# Make Python scripts executable
RUN chmod +x /home/mavros_navigator/scripts/*.py

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /home/mavros_navigator/src/harpia_msgs && \
    colcon build && \
    source install/setup.bash && \
    cd /home/mavros_navigator/ && \
    colcon build && \
    source install/setup.bash && \
    chmod +x install/route_executor2/share/route_executor2/solver/OPTIC/generate_plan.sh install/route_executor2/share/route_executor2/solver/OPTIC/optic-clp && \
    chmod +x install/route_executor2/share/route_executor2/solver/TFD/generate_plan.sh install/route_executor2/share/route_executor2/solver/TFD/downward/preprocess/preprocess install/route_executor2/share/route_executor2/solver/TFD/downward/search/search"

COPY container_scripts/ /home/scripts

# # Copy and setup entrypoint
COPY entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh

# RUN rm -f /home/mavros_navigator/entrypoint.sh
# RUN echo "#!/bin/bash\nls\nsleep 5" > /home/mavros_navigator/entrypoint.sh && chmod +x /home/mavros_navigator/entrypoint.sh



ENTRYPOINT ["/entrypoint.sh"]