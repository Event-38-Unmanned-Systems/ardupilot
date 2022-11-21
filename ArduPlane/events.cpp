#include "Plane.h"

void Plane::failsafe_short_on_event(enum failsafe_state fstype, ModeReason reason)
{
    // This is how to handle a short loss of control signal failsafe.
    failsafe.state = fstype;
    failsafe.short_timer_ms = millis();
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event on: type=%u/reason=%u", fstype, static_cast<unsigned>(reason));
    switch (control_mode->mode_number())
    {
    case Mode::Number::MANUAL:
    case Mode::Number::STABILIZE:
    case Mode::Number::ACRO:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
    case Mode::Number::TRAINING:
        failsafe.saved_mode_number = control_mode->mode_number();
        failsafe.saved_mode_set = true;
        if(g.fs_action_short == FS_ACTION_SHORT_FBWA) {
            set_mode(mode_fbwa, reason);
        } else {
            set_mode(mode_circle, reason);
        }
        break;

    case Mode::Number::QSTABILIZE:
    case Mode::Number::QLOITER:
    case Mode::Number::QHOVER:
    case Mode::Number::QAUTOTUNE:
    case Mode::Number::QACRO:
        failsafe.saved_mode_number = control_mode->mode_number();
        failsafe.saved_mode_set = true;
        if (quadplane.options & QuadPlane::OPTION_FS_QRTL) {
            set_mode(mode_qrtl, reason);
        } else {
            set_mode(mode_qland, reason);
        }
        break;
        
    case Mode::Number::AUTO:
	if (plane.auto_state.wp_is_land_approach || ((quadplane.fs_wait_start + (quadplane.fs_wait*1000)) > millis()) || quadplane.in_transition()){
		break;
	}
	if(quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)){
		set_mode(mode_qland, reason);
		break;
	}
	else FALLTHROUGH;
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::LOITER:
        if(g.fs_action_short != FS_ACTION_SHORT_BESTGUESS) {
            failsafe.saved_mode_number = control_mode->mode_number();
            failsafe.saved_mode_set = true;
            if(g.fs_action_short == FS_ACTION_SHORT_FBWA) {
                set_mode(mode_fbwa, reason);
            } else {
                set_mode(mode_circle, reason);
            }
        }
        break;

    case Mode::Number::CIRCLE:
    case Mode::Number::TAKEOFF:
    case Mode::Number::RTL:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::INITIALISING:
        break;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode->mode_number());
}

void Plane::failsafe_long_on_event(enum failsafe_state fstype, ModeReason reason)
{
    // This is how to handle a long loss of control signal failsafe.
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event on: type=%u/reason=%u", fstype, static_cast<unsigned>(reason));
    //  If the GCS is locked up we allow control to revert to RC
    RC_Channels::clear_overrides();

    switch (control_mode->mode_number())
    {
    case Mode::Number::MANUAL:
    case Mode::Number::STABILIZE:
    case Mode::Number::ACRO:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
    case Mode::Number::TRAINING:
    case Mode::Number::CIRCLE:
		failsafe.state = fstype;
        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(mode_fbwa, reason);
        } else {
            set_mode(mode_rtl, reason);
        }
        break;

    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QACRO:
    case Mode::Number::QAUTOTUNE:
	        failsafe.state = fstype;
        if (quadplane.options & QuadPlane::OPTION_FS_QRTL) {
            set_mode(mode_qrtl, reason);
        } else {
            set_mode(mode_qland, reason);
        }
        break;
        
    case Mode::Number::AUTO:
	if (plane.auto_state.wp_is_land_approach || ((quadplane.fs_wait_start + (quadplane.fs_wait*1000)) > millis()) || quadplane.in_transition()){
		break;
	}
		if(quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)){
		set_mode(mode_qland, reason);
		break;
	}
	else FALLTHROUGH;
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::LOITER:
	    failsafe.state = fstype;
        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(mode_fbwa, reason);
        } else if (g.fs_action_long == FS_ACTION_LONG_RTL) {
            set_mode(mode_rtl, reason);
        }
        break;

    case Mode::Number::RTL:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::TAKEOFF:
    case Mode::Number::INITIALISING:
	    failsafe.state = fstype;
        break;
    }
	if (failsafe.state  == fstype){
    gcs().send_text(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode->mode_number());
	}
}

void Plane::failsafe_short_off_event(ModeReason reason)
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event off: reason=%u", static_cast<unsigned>(reason));
    failsafe.state = FAILSAFE_NONE;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (control_mode == &mode_circle && failsafe.saved_mode_set) {
        failsafe.saved_mode_set = false;
        set_mode_by_number(failsafe.saved_mode_number, reason);
    }
}

void Plane::failsafe_long_off_event(ModeReason reason)
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event off: reason=%u", static_cast<unsigned>(reason));
    failsafe.state = FAILSAFE_NONE;
}

//
bool Plane::canFailsafe()
{
	
	if (plane.auto_state.wp_is_land_approach && control_mode == &mode_auto){
		return false;
	}
	if (control_mode == &mode_qland){
		return false;
	}
	if (quadplane.in_vtol_land_sequence()){
		return false;
	}
	
		return true;	
}

//enacts failsafe option passed in through ACTION
void Plane:: handle_failsafe_switch(const char *type_str,const int8_t action){
 switch ((Failsafe_Action)action) {
        case Failsafe_Action_QLand:
            if (quadplane.available()) {
                plane.set_mode(mode_qland, ModeReason::BATTERY_FAILSAFE);
                break;
            }
            FALLTHROUGH;
        case Failsafe_Action_Land:
                if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND) {
                // never stop a landing if we were already committed
                if (plane.mission.jump_to_landing_sequence()) {
                    plane.set_mode(mode_auto, ModeReason::BATTERY_FAILSAFE);
                    break;
                }
            }
            FALLTHROUGH;
        case Failsafe_Action_RTL:
            if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND) {
                // never stop a landing if we were already committed
                set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE);
                aparm.throttle_cruise.load();
            }
            break;

	
        case Failsafe_Action_Terminate:
#if ADVANCED_FAILSAFE == ENABLED
            char battery_type_str[17];
            snprintf(battery_type_str, 17, "%s battery", type_str);
            afs.gcs_terminate(true, battery_type_str);
#else
            arming.disarm();
#endif
            break;

        case Failsafe_Action_Parachute:
#if PARACHUTE == ENABLED
            parachute_release();
#endif
            break;

        case Failsafe_Action_None:
            // don't actually do anything, however we should still flag the system as having hit a failsafe
            // and ensure all appropriate flags are going off to the user
            break;
		}
}

//handles critical vs low battery failsafe actions
void Plane::handle_battery_failsafe(const char *type_str, const int8_t action)
{	
     //handle low battery action
	if (strcmp(type_str,"low") == 0 && canFailsafe()){
		
		//if quadplane is not enabled we handle failsafe the default way
		if (!quadplane.available() || (quadplane.options & quadplane.OPTION_OLD_FS_BATT)) {
		     handle_failsafe_switch(type_str,action);
		}
		//if quadplane is available and we're in a vtol mode and not in a vtol manual mode we perform user selected failsafe
	    //if we're in QRTL we're likely having an issue transitioning we don't want to force a transition.
		else if(quadplane.available() && quadplane.in_vtol_mode() && !quadplane.in_manual_vtol_mode() && control_mode != &mode_qrtl) {
			// handles vtol takeoff 
		if(quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id) && (plane.control_mode == &plane.mode_auto)) {
		    handle_failsafe_switch(type_str,(Failsafe_Action)Failsafe_Action_QLand);
			gcs().send_text(MAV_SEVERITY_WARNING, "Low battery during takeoff QLAND");			
		}
		//if in multicopter mode we perform a transition
		else{
		handle_failsafe_switch(type_str,action);
		quadplane.preventFWtoVTOLTransition = true;			
		gcs().send_text(MAV_SEVERITY_WARNING, "Low battery aircraft will only transition on landing or manual command");
		}
		}
		//if quadplane is in fixed wing flight and we hit low battery failsafe we notify user 
		//and no longer accept do_vtol_transition commands 
		//to switch from fixed wing to VTOL flight in missions.
		//this still allows user switch in manual modes 
		else if(quadplane.available() && !quadplane.in_vtol_mode()) {
			handle_failsafe_switch(type_str,(Failsafe_Action)Failsafe_Action_None);
			quadplane.preventFWtoVTOLTransition = true;	
			gcs().send_text(MAV_SEVERITY_WARNING, "Low battery aircraft will only transition on landing or manual command");
		}
		//we should be in a manual vtol mode or QRTL here do nothing and log
		else {
			handle_failsafe_switch(type_str,(Failsafe_Action)Failsafe_Action_None);		
			quadplane.preventFWtoVTOLTransition = true;			
			gcs().send_text(MAV_SEVERITY_WARNING, "Low battery aircraft will only transition on landing or manual command");
		}			
						
	}
		
		if (strcmp(type_str,"critical") == 0 && canFailsafe()){
		//if quadplane is in a manual flight mode we switch to qland 
		//handle battery failsafe in vtol takeoff
		if (quadplane.available() && ((quadplane.in_manual_vtol_mode() || control_mode == &mode_qrtl) || (quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id) && (plane.control_mode == &plane.mode_auto)) ) )
		{
		plane.set_mode(mode_qland, ModeReason::BATTERY_FAILSAFE);
		}
		//otherwise we handle failsafes normally
		else{
		   handle_failsafe_switch(type_str, action);
	}	
}

}
