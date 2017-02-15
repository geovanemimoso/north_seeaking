require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'

describe 'north_seeking::Task configuration' do

    include Orocos::Test::Component
    start 'north_seeking', 'north_seeking::Task' => 'north_seeking'
    reader 'north_seeking', 'yaw_rotation', attr_name: 'yaw_rotation'
    writer 'north_seeking', 'pose_samples', attr_name: 'pose_samples'
    writer 'north_seeking', 'gps_position_samples', attr_name: 'gps_position_samples'
