#include "../src/perception_view.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>
void check(bool v){if(!v)throw std::runtime_error("perception view freshness or unknown semantics failed");}
int main(){
  auv_runtime::PerceptionView v;v.sequence=v.tag_sequence=1;v.stamp=v.tag_stamp=v.pose_stamp=10;
  v.pose_valid=true;v.row=v.col=1.5;v.reason="ambiguity\n\"unknown\"";v.jpeg={0,1,2,3};
  auv_vision::AprilTagObservation tag;tag.id=18;v.tags.push_back(tag);v.visible[0]=true;
  auto fresh=YAML::Load(v.json(10.1,.3,.5));check(fresh["pose_valid"].as<bool>()&&fresh["tags"].size()==1);
  check(fresh["cells"][0]["type"].as<std::string>()=="unknown");
  check(!fresh["cells"][0]["observed"].as<bool>());
  check(fresh["cells"][0]["coverage_fraction"].as<double>()==0);
  v.cells[0].visible_views.fill(3);v.cells[0].empty_views.fill(3);
  auto covered=YAML::Load(v.json(10.1,.3,.5));
  check(covered["cells"][0]["coverage_fraction"].as<double>()==1&&
    covered["cells"][0]["empty_support_fraction"].as<double>()==1);
  check(covered["cells"][0]["type"].as<std::string>()=="unknown"); // Diagnostic coverage is not map authority.
  check(fresh["image_url"].as<std::string>()=="data:image/jpeg;base64,AAECAw==");
  auto old=YAML::Load(v.json(11,.3,.5));check(!old["pose_valid"].as<bool>()&&old["tags"].size()==0);
  auto dead=YAML::Load(v.json(12,.3,.5));check(!dead["frame_fresh"].as<bool>()&&dead["image_url"].as<std::string>().empty());
  check(!dead["cells"][0]["visible_now"].as<bool>());
  auto future=YAML::Load(v.json(9,.3,.5));check(!future["pose_valid"].as<bool>()&&future["tags"].size()==0);
  cv::Mat image(240,320,CV_8UC3,cv::Scalar(30,50,40));
  cv::rectangle(image,{20,50,40,80},cv::Scalar(0,0,220),cv::FILLED);
  v.attach_image(image,{},false);check(v.red_components==1&&!v.jpeg.empty());
  check(!cv::imdecode(v.jpeg,cv::IMREAD_COLOR).empty());
  v.row=NAN;check(!YAML::Load(v.json(10.1,.3,.5))["pose_valid"].as<bool>());
}
