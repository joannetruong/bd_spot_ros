conda create -n kin2dyn -y python=3.8 && \
conda activate kin2dyn && \
conda install -y mamba -c conda-forge && \
conda config --env --add channels conda-forge && \
conda config --env --add channels robostack && \
conda config --env --set channel_priority strict && \
mamba install -y ros-noetic-desktop && \
conda deactivate && \
conda activate kin2dyn && \
mkdir kin2dyn && cd kin2dyn && \
git clone git@github.com:joannetruong/bd_spot_ros.git  && \
git clone --branch outdoor_nav_real git@github.com:joannetruong/habitat-lab.git && \
git clone --branch outdoor_nav git@github.com:joannetruong/spot_rl_experiments.git && \
pip install -e . && \
pip install -r requirements.txt && \
mamba install -y ros-noetic-realsense2-camera && \
cd ../habitat-lab && \
pip install typing-extensions~=3.7.4 google-auth==1.6.3 simplejson braceexpand pybullet cython pkgconfig tensorboard blosc && \
pip install -r requirements.txt && \
python setup.py develop --all && \
echo 'export LD_LIBRARY_PATH=/home/spot/anaconda3/envs/kin2dyn/lib' >> ~/.bashrc

