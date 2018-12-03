# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  
  config.vm.box = "ubuntu/xenial64"
  
  # use shell and other provisioners as usual
  config.vm.provision :shell, path: "build_steps/provision.sh"
  
  config.vm.provider "virtualbox" do |v|
    v.memory = 10000
    v.cpus = 4
    v.customize [ "guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-threshold", 10000 ]
  end 
  config.vm.provision "shell" do |s|
    s.inline = <<-SCRIPT
      # Change directory automatically on ssh login
      echo "cd    /vagrant" >> /home/vagrant/.bash_profile
      echo "source build_steps/environment.sh" >> /home/vagrant/.bash_profile
    SCRIPT
  end
end
