
FROM ubuntu:18.04

WORKDIR /opt
COPY . /opt

USER root

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get -o Acquire::Check-Valid-Until=false -o Acquire::Check-Date=false update -y
RUN apt-get install -y python3.6 \
                       python3-pip \
                       wget \
                       build-essential \
                       software-properties-common \
                       apt-utils \
                       libgl1-mesa-glx \
                       ffmpeg \
                       libsm6 \
                       libxext6 \
                       libffi-dev \
                       libbz2-dev \
                       zlib1g-dev \
                       libreadline-gplv2-dev \
                       libncursesw5-dev \
                       libssl-dev \
                       libsqlite3-dev \
                       tk-dev \
                       libgdbm-dev \
                       libc6-dev \
                       liblzma-dev

# RUN wget https://www.python.org/ftp/python/3.6.13/Python-3.6.13.tgz
# RUN tar -xzf Python-3.6.13.tgz
# RUN cd Python-3.6.13/ && ./configure --with-ensurepip=install && make && make install

RUN apt-get update
#RUN apt-get install -y python3-pip
RUN pip3 install -r requirements.txt
RUN apt-get install -y locales && locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

ENTRYPOINT [ "python3", "/opt/3d_icp_merge_east_west.py" ]
