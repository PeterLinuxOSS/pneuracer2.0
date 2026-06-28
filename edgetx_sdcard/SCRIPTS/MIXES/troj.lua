local switch_name = "se"
local timeout_ticks = 100 -- 1 sekunda
local last_state = 0
local changes = 0
local last_time = 0
local toggle_gv1 = 0
local toggle_gv2 = 0

local function init()
    last_state = getValue(switch_name)
    last_time = getTime()
end

local function run()
    local current_state = getValue(switch_name)
    local current_time = getTime()

    -- 1. Časový limit vypršal - vyhodnoť čo bolo naklikané
    if changes > 0 and (current_time - last_time) > timeout_ticks then
        if changes >= 8 then
            -- 4+ kliky = toggle GV2
            toggle_gv2 = 1 - toggle_gv2
        elseif changes >= 6 then
            -- presne 3 kliky = toggle GV1
            toggle_gv1 = 1 - toggle_gv1
        end
        changes = 0
    end

    -- 2. Detekcia pohybu spínača
    if current_state ~= last_state then
        changes = changes + 1
        last_state = current_state
        last_time = current_time
    end

    -- 3. Zápis do GV1 a GV2
    model.setGlobalVariable(0, 0, toggle_gv1)
    model.setGlobalVariable(1, 0, toggle_gv2)

    return 0
end

return { init=init, run=run }