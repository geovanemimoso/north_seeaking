require 'orocos'
require 'vizkit'
require "transformer/runtime"
include Orocos

address = ARGV[0].to_s
#address1 = ARGV[1].to_s

#@log_replay = Orocos::Log::Replay.open(address, address1)
@log_replay = Orocos::Log::Replay.open(address)


#######################################################################

Orocos.run 'north_seeking::Task' => 'north_seeking' do

    north_seeking     = TaskContext.get 'north_seeking'

	Orocos.log_all

    #########################################################
	gps	 = @log_replay.ecomm_board
	dead = @log_replay.dead_reckoning

    #########################################################
    ##Orocos.transformer.load_conf("static_transforms.rb")
    ##Orocos.transformer.setup(north_seeking)
    north_seeking.apply_conf_file('north_seeking::Task.yml')
    ##########################################################################

    gps.position_samples.connect_to		north_seeking.gps_position_samples,    :type => :buffer, :size => 100
    dead.pose_samples.connect_to	north_seeking.pose_samples,	               :type => :buffer, :size => 100

    north_seeking.configure
    north_seeking.start

    Vizkit.control @log_replay
    Vizkit.exec
end
