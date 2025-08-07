cp2pi() {
  scp $1 robot@192.168.1.100:$2
}

cp2pir() {
	scp -r $1 robot@192.168.1.100:$2 && ssh robot@192.168.1.100 "cd /home/robot/maze-runner/ros;source ./install/setup.bash;source ./install/local_setup.bash;colcon build"
}

genssh() {
	ssh-keygen -t rsa
	cat ~/.ssh/id_rsa.pub | ssh robot@192.168.1.100 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
}

