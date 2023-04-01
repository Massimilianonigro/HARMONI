# HARMONI openface


## Setup

First install clone the openface directory:

```bash 
git clone https://github.com/TadasBaltrusaitis/OpenFace.git
bash download_models.sh
``` 

Then run the following commands in the harmoni_detector container (pick this container to avoid messing up with libraries installed for other services)


```bash 
cd OpenFace
apt-get update -qq &&\
apt-get install -qq curl &&\
apt-get install -qq --no-install-recommends \
    libopenblas-dev liblapack-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libtbb2 libtbb-dev libjpeg-dev \
    libpng-dev libtiff-dev &&\
rm -rf /var/lib/apt/lists/*

apt-get update -qq && apt-get install -qq -y \
        cmake ninja-build pkg-config build-essential checkinstall\
        g++-8 &&\
rm -rf /var/lib/apt/lists/* &&\
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8

curl http://dlib.net/files/dlib-19.13.tar.bz2 -LO &&\
tar xf dlib-19.13.tar.bz2 && \
rm dlib-19.13.tar.bz2 &&\
mv dlib-19.13 dlib &&\
mkdir -p dlib/build &&\
cd dlib/build &&\
cmake -DCMAKE_BUILD_TYPE=Release -G Ninja .. &&\
ninja && \
ninja install && \
DESTDIR=/root/diff ninja install &&\
ldconfig

OPENCV_VERSION=4.1.0

curl https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz -LO &&\
tar xf ${OPENCV_VERSION}.tar.gz && \
rm ${OPENCV_VERSION}.tar.gz &&\
mv opencv-${OPENCV_VERSION} opencv && \
mkdir -p opencv/build && \
cd opencv/build && \
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_TBB=ON -D WITH_CUDA=OFF \
    -DWITH_QT=OFF -DWITH_GTK=OFF\
    -G Ninja .. && \
ninja && \
ninja install &&\
DESTDIR=/root/diff ninja install


``` 
And then (if necessary remove the CMakeCache.txt):


``` 
mkdir -p build && cd build && \
cmake -D CMAKE_BUILD_TYPE=RELEASE -G Ninja .. && \
ninja &&\
DESTDIR=/root/diff ninja install

ldconfig
``` 
## Usage

Run the following commands in order to run camera service and openface service in two difopenfaceent terminals:

```  bash
roslaunch harmoni_sensors camera_service.launch
roslaunch harmoni_openface openface_service.launch
```

## Testing

Camera will open and one frame will be captured and put as input of the openface model. Once the output of the openface service arrives another frame will be captured and again used as input for openface model and so on. 
