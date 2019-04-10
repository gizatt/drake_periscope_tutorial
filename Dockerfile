FROM ubuntu:16.04

ARG DRAKE_URL

RUN apt-get update
RUN apt install -y sudo graphviz python-pip curl git
RUN pip install pip ipykernel==4.10.0 ipython==5.5.0 jupyter graphviz meshcat numpy "tornado<6,>5"
RUN curl -o drake.tar.gz $DRAKE_URL && sudo tar -xzf drake.tar.gz -C /opt
RUN yes | sudo /opt/drake/share/drake/setup/install_prereqs
RUN git clone https://github.com/RussTedrake/underactuated /underactuated && cd /underactuated && git checkout 17687cb52ff8febd77a8f881729317dff3ee8c67
RUN yes | sudo /underactuated/scripts/setup/ubuntu/16.04/install_prereqs
RUN apt install -y python-tk xvfb mesa-utils libegl1-mesa libgl1-mesa-glx libglu1-mesa libx11-6 x11-common x11-xserver-utils

ENV PYTHONPATH=/underactuated/src:/opt/drake/lib/python2.7/site-packages
COPY ./ /test_dir

ENTRYPOINT bash -c "/test_dir/run_tests.sh"
