// Included ONLY in the test executable, while holding Runtime::state_mutex_.
// Supplies explicitly synthetic stage preconditions, never a measured pose,
// ARM acknowledgement, controller result or fabricated completion event.
if(cmd=="stage-test-freeze-map"||cmd=="stage-test-after-ascent"||cmd=="stage-test-after-cones") {
  if(cfg_.operation_mode!="debug"||!stage_virtual_device(cfg_.serial)||cfg_.auto_arm||
      cfg_.auto_start||cfg_.web_enabled||!cfg_.local_mapping_enabled||!safe_status(now)||
      !depth_sample_fresh(now)||!status_.armed||!armed_requested_)
    return "ERR synthetic stage fixture safety guard\n";
  const auto phase=mission_.snapshot().phase;
  if(cmd=="stage-test-freeze-map") {
    if(phase!=auv_mission::MissionPhase::kBuildMap||!tag_found_||!metric_pose_.valid||
        !metric_pose_.landmark_observation||now<metric_pose_.stamp||now-metric_pose_.stamp>.3)
      return "ERR fresh BUILD_MAP landmark required for stage fixture\n";
    auv_core::SemanticMap fixture;fixture.complete=fixture.grid.complete=true;
    fixture.grid.rows=fixture.grid.cols=3;fixture.cone_count=4;
    for(int row=0;row<3;++row)for(int col=0;col<3;++col) {
      const auto type=(col==0||col==2)?(row==0?"circle_cone":row==2?"square_cone":"empty"):"empty";
      fixture.grid.cells.push_back({{static_cast<std::int8_t>(row),static_cast<std::int8_t>(col),type},false});
    }
    map_=std::move(fixture);
    // Preserve the supplied predecessor's actually received pressure datum
    // while entering the next stage; never invent a sensor reading.
    hold_depth_=depth_sample_.depth_m;
    event("STAGE_FIXTURE","synthetic frozen 3x3 map; BUILD_MAP acceptance explicitly excluded");
  } else if(cmd=="stage-test-after-ascent") {
    if(phase!=auv_mission::MissionPhase::kSurfaceForCones||!map_.complete||
        std::abs(depth_sample_.depth_m-cfg_.traversal.surface_depth_m)>.01)
      return "ERR supplied post-ascent pressure precondition unavailable\n";
    ascent_started_=surface_confirmed_=true;hold_depth_=static_cast<float>(cfg_.traversal.surface_depth_m);
    surface_reacquire_start_=now;surface_vote_sequence_=0;surface_pose_votes_=0;
    metric_pose_={};pose_valid_=false;surface_ready_=false;
    mission_.update_surface(true,now);
    event("STAGE_FIXTURE","synthetic post-ascent entry; pressure ascent acceptance explicitly excluded");
  } else {
    if(phase!=auv_mission::MissionPhase::kVisitCones||!surface_grid_fresh(now)||!map_.complete)
      return "ERR fresh VISIT_CONES precondition required for exit fixture\n";
    const int cell=int(metric_pose_.row)*3+int(metric_pose_.col);
    if(cell!=0&&cell!=2&&cell!=6&&cell!=8)return "ERR exit fixture must start in supplied last cone cell\n";
    for(int i:{0,2,6,8})visited_[i]=true;
    all_visited_=true;
    event("STAGE_FIXTURE","synthetic four visited cones; measured cone visitation acceptance explicitly excluded");
  }
  return "OK synthetic stage precondition supplied\n";
}
