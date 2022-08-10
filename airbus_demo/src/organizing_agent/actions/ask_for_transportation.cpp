/*
    implementation for this action



    if progress == 0
        picked_agents_ = select_rnd_agents(num)
        for all picked_agents_:
            accepted, performed = sendBeliefUpd tool_required
            accepted, performed = sendBeliefUpd payload_in (i.e. where it is now)
            
            if not accepted or not performed:
                FAILURE
            else:
                accepted, performed = sendDesireReq with value: 
                    [payload_in (where it should go), ***something to say that you need to wait and do the job with others***]
                if not accepted or not performed:
                    FAILURE
                    
    else:
        target_reached = .... #isMonitoredDesireFulfilled true for all the monitored desires

        if(target_reached)
            SUCCESS
        elif(progress == 99 and not target_reached):
            FAILURE

    
    return tiny_increment 

*/