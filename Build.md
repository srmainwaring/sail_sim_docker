# Build

Building:

```bash
git clone ssh://rhys@diskstation.local:/volume1/git/robotics/sail_sim_docker.git
cd sail_sim_docker && checkout develop

mkdir src && cd src
git clone ssh://rhys@diskstation.local:/volume1/git/robotics/asv_sim.git
git clone ssh://rhys@diskstation.local:/volume1/git/robotics/asv_wave_sim.git
git clone ssh://rhys@diskstation.local:/volume1/git/robotics/rs750.git

cd ~/sail_sim_docker/src/rs750
git checkout feature/controller

cd ~/sail_sim_docker/src/asv_sim
git checkout feature/docker

cd ~/sail_sim_docker/src/asv_wave_sim
git checkout feature/fft_waves

cd ~/sail_sim_docker
sudo docker build -t rhysmainwaring/sail-sim .
```

Running (insecure):

```bash
# !!! WARNING !!!
xhost +

# Start the containers
docker-compose -f docker-compose-vmware.yaml up
```
