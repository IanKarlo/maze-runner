cp2pi() {
  scp $1 robot@192.168.1.100 $2
}

genssh() {
	ssh-keygen -t rsa
	cat ~/.ssh/id_rsa.pub | ssh robot@192.168.1.100 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
}

