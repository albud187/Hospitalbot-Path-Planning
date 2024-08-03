#use osrf as starting point since it already comes with python3 / c++ / numpy /etc
FROM osrf/ros:humble-desktop

RUN apt update
RUN apt install nano

RUN apt install -y python3-pip

COPY requirements.txt requirements.txt
RUN python3 -m pip install -r requirements.txt

RUN pip install "numpy<2"

RUN apt update && \
    apt install -y xterm

ENTRYPOINT ["/bin/bash", "-c", "bash entrypoint.sh && exec bash"]