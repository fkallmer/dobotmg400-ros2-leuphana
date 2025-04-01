FROM ghcr.io/sloretz/ros:humble-desktop-full

# Systempakete aktualisieren, notwendige Pakete installieren und aufräumen
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        python3-pip \
        nlohmann-json3-dev \
        git \
        build-essential \
        cmake && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Paho MQTT C++ Bibliothek installieren
WORKDIR /tmp
RUN git clone https://github.com/eclipse/paho.mqtt.cpp.git && \
    cd paho.mqtt.cpp && \
    git checkout v1.5.2 && \
    git submodule update --init && \
    cmake -S . -B build -DPAHO_WITH_MQTT_C=ON -DPAHO_BUILD_EXAMPLES=ON && \
    cmake --build build --target install && \
    cd / && \
    rm -rf /tmp/paho.mqtt.cpp

# Setze den Arbeitsordner
WORKDIR /root

CMD ["bash"]
