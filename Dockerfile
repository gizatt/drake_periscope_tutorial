FROM ubuntu:16.04

ARG DRAKE_URL

RUN apt-get update
RUN apt install -y sudo graphviz python-pip curl git
RUN pip install --upgrade pip jupyter graphviz meshcat numpy matplotlib
RUN curl -o drake.tar.gz $DRAKE_URL && sudo tar -xzf drake.tar.gz -C /opt
RUN yes | sudo /opt/drake/share/drake/setup/install_prereqs
RUN git clone https://github.com/RussTedrake/underactuated /underactuated
RUN yes | sudo /underactuated/scripts/setup/ubuntu/16.04/install_prereqs
RUN apt install -y python-tk xvfb

ENV PYTHONPATH=/underactuated/src:/opt/drake/lib/python2.7/site-packages
COPY ./ /test_dir

ENTRYPOINT bash -c "/test_dir/run_tests.sh"
