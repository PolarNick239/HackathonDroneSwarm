## Possible states

- flyToMission (to target point)
- onMission (executing mission)
- flyToCharge (fly to nearest charging station)
- onCharge
- wait (do nothing)

## Possible states transitions

- flyToMission -> onMission
- flyToMission/onMission/wait -> flyToCharge
- flyToCharge -> onCharge
- onMission -> wait
- onCharge -> flyToMission/wait
