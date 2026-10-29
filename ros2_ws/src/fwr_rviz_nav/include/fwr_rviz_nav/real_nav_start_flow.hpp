#pragma once

namespace fwr_rviz_nav
{
class RealNavStartFlow
{
public:
  enum class Stage {idle, opening_gate, waiting_ready, arming, starting_goal, active};

  bool begin()
  {
    if (stage_ != Stage::idle) {return false;}
    saw_running_state_ = false;
    stage_ = Stage::opening_gate;
    return true;
  }

  bool gate_opened(bool success)
  {
    if (stage_ != Stage::opening_gate) {return false;}
    stage_ = success ? Stage::waiting_ready : Stage::idle;
    return success;
  }

  bool ready()
  {
    if (stage_ != Stage::waiting_ready) {return false;}
    stage_ = Stage::arming;
    return true;
  }

  bool armed(bool success)
  {
    if (stage_ != Stage::arming) {return false;}
    stage_ = success ? Stage::starting_goal : Stage::idle;
    return success;
  }

  bool goal_accepted(bool accepted)
  {
    if (stage_ != Stage::starting_goal) {return false;}
    stage_ = accepted ? Stage::active : Stage::idle;
    return accepted;
  }

  bool active() const {return stage_ == Stage::active;}
  bool saw_running_state() const {return saw_running_state_;}
  bool explore_state(bool stopped)
  {
    if (!active()) {return false;}
    if (!stopped) {saw_running_state_ = true;}
    return stopped && saw_running_state_;
  }
  bool waiting_ready() const {return stage_ == Stage::waiting_ready;}
  bool pending_or_active() const {return stage_ != Stage::idle;}

  bool stop()
  {
    const bool was_pending_or_active = pending_or_active();
    stage_ = Stage::idle;
    saw_running_state_ = false;
    return was_pending_or_active;
  }

private:
  Stage stage_ = Stage::idle;
  bool saw_running_state_ = false;
};
}  // namespace fwr_rviz_nav
