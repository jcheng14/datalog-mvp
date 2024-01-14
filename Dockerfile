FROM ubuntu:focal

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y locales
ENV LANG="en_US.UTF-8" LC_ALL="en_US.UTF-8" LANGUAGE="en_US.UTF-8"
RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
    locale-gen --purge $LANG && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    update-locale LANG=$LANG LC_ALL=$LC_ALL LANGUAGE=$LANGUAGE

ENV TZ=America/Los_Angeles

# Setting up workspace for workdir
ENV WORKSPACE="/workspace/"

RUN apt update && apt install  --no-install-recommends -y \
    build-essential \
    clang \
    cmake \
    curl \
    debconf-utils \
    gcc \
    gdb \
    git \
    libgl1 \
    libgomp1 \
    libpcl-dev \
    libx11-dev\
    nano \
    python3.8 \
    python3-pip \
    qt5-default \
    wget && \
    apt clean && rm -rf /var/lib/apt/lists/*
     
# Ref: https://github.com/Livox-SDK/Livox-SDK
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    cd build && cmake .. && \
    make && \
    make install

# Ref: Visualizing pcd files and other pcd operations
RUN python3 -m pip install --no-cache-dir --upgrade pip>=20.3 && \
    python3 -m pip install open3d 

# Ref: https://github.com/klintan/pypcd 
# For converting cvs to pcd 
RUN python3 -m pip install --upgrade git+https://github.com/klintan/pypcd.git

ARG LIVOX_VIEWER_URL=https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Download/update/Livox_Viewer_For_Linux_Ubuntu16.04_x64_0.10.0.tar.gz
ARG LIVOX_VIEWER=Livox_Viewer_For_Linux_Ubuntu16.04_x64_0.10.0.tar.gz

RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${LIVOX_VIEWER_URL} -O ${LIVOX_VIEWER} && \
    tar -xzvf ${LIVOX_VIEWER}

COPY . .

# Install the Flir Camera SDK
RUN cd spinnaker-3.0.0.118-amd64 && \
    echo 'libspinnaker libspinnaker/accepted-flir-eula boolean true'| debconf-set-selections && \
    dpkg -i libgentl_*.deb && \
    dpkg -i libspinnaker_*.deb && \
    dpkg -i libspinnaker-dev_*.deb && \
    dpkg -i libspinnaker-c_*.deb && \
    dpkg -i libspinnaker-c-dev_*.deb && \
    dpkg -i libspinvideo_*.deb && \
    dpkg -i libspinvideo-dev_*.deb && \
    dpkg -i libspinvideo-c_*.deb && \
    dpkg -i libspinvideo-c-dev_*.deb && \
    apt-get install -y ./spinview-qt_*.deb && \
    dpkg -i spinview-qt-dev_*.deb && \
    dpkg -i spinupdate_*.deb && \
    dpkg -i spinupdate-dev_*.deb && \
    dpkg -i spinnaker_*.deb && \
    dpkg -i spinnaker-doc_*.deb

# Install Python Wrapper for Flir Camera SDK 
RUN cd spinnaker_python-3.0.0.118-cp38-cp38-linux_x86_64 && \
    python3 -m pip install spinnaker_python-3.0.0.118-cp38-cp38-linux_x86_64.whl \
    keyboard 

RUN python3 -m pip install opencv-python \
    black \   
    flake8 \
    pyqt5 \
    scikit-learn 

ENV XDG_RUNTIME_DIR=/tmp

WORKDIR ${WORKSPACE} 
