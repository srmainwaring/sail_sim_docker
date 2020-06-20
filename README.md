# Docker file for Sailing Simulation

Docker file and scripts.

- [wilselby/ouster_example](https://github.com/wilselby/ouster_example).


Building:

```bash
sudo docker build -t sail_sim .
```

Running (recklessly insecure):

```bash
# !!!!!!!!!
xhost +

sudo docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name sail_sim sail_sim
```

```bash
sudo docker exec -it sail_sim bash
```

## Running GUIs from Docker

- [Can you run GUI applications in a Docker container?](https://stackoverflow.com/questions/16296753/can-you-run-gui-applications-in-a-docker-container/25280523#25280523)
- [Running GUI apps with Docker](http://fabiorehm.com/blog/2014/09/11/running-gui-apps-with-docker/)


Security Concerns:

- Sharing X11 info with docker allows the application within docker to receive every X11 event in the
desktop, including all keys typed in all other apps.

Suggested solutions:

- x11docker [https://github.com/mviereck/x11docker](https://github.com/mviereck/x11docker)


macOS:

- [Running GUI Apps in Docker on Mac OS X](https://darrensnotebook.blogspot.com/2016/04/running-gui-apps-in-docker-on-mac-os-x.html)
- [how to use -e DISPLAY flag on osx?](https://github.com/moby/moby/issues/8710)
- [BWC: GUI apps in Docker on OSX](http://blog.bennycornelissen.nl.s3-website-eu-west-1.amazonaws.com/post/bwc-gui-apps-in-docker-on-osx/)

