#unique identifier for a specific bot
string bot_id

#one of the following: 'load', 'transport', 'charge', 'move', 'defect'
string bot_status

#textual description of the bot, does not have to be unique, might be used for Identification or types of bots
string bot_name

#load status of powers supply in % fro 0 .. 100
uint8 bot_akku_status

#unique identifier for specific tool attached to the bot
string bot_attached_tool_id

#textual description of the tool attached to the bot, does not have to be unique, might be used for Identification or types of tools
string bot_attached_tool_name

#unique identifier of actual job the bot is integrated to
string bot_actual_job_id

#unique identifier of nexct planned job for the bot
string bot_next_planned_job_id

#actual role of the bot for actual job, one of the following: 'master', 'slave'
string bot_actual_role

#actual name of master for bot in case bot is slave to a job, otherwise other content
string bot_actual_team_leader

#tbd 3 or 6 DOF (3 should be sufficient)
float32[] bot_position_orientation
