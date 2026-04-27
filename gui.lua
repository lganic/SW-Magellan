--- Developed using LifeBoatAPI - Stormworks Lua plugin for VSCode - https://code.visualstudio.com/download (search "Stormworks Lua with LifeboatAPI" extension)
--- If you have any issues, please report them here: https://github.com/nameouschangey/STORMWORKS_VSCodeExtension/issues - by Nameous Changey


--[====[ HOTKEYS ]====]
-- Press F6 to simulate this file
-- Press F7 to build the project, copy the output from /_build/out/ into the game to use
-- Remember to set your Author name etc. in the settings: CTRL+COMMA


--[====[ EDITABLE SIMULATOR CONFIG - *automatically removed from the F7 build output ]====]
---@section __LB_SIMULATOR_ONLY__
do
    ---@type Simulator -- Set properties and screen sizes here - will run once when the script is loaded
    simulator = simulator
    simulator:setScreen(1, "1x1")
    -- simulator:setProperty("ExampleNumberProperty", 123)

    -- Runs every tick just before onTick; allows you to simulate the inputs changing
    ---@param simulator Simulator Use simulator:<function>() to set inputs etc.
    ---@param ticks     number Number of ticks since simulator started
    function onLBSimulatorTick(simulator, ticks)

        -- touchscreen defaults
        local screenConnection = simulator:getTouchScreen(1)
        simulator:setInputBool(1, screenConnection.isTouched)
        simulator:setInputNumber(1, screenConnection.width)
        simulator:setInputNumber(2, screenConnection.height)
        simulator:setInputNumber(3, screenConnection.touchX)
        simulator:setInputNumber(4, screenConnection.touchY)

        -- -- NEW! button/slider options from the UI
        -- simulator:setInputBool(31, simulator:getIsClicked(1))       -- if button 1 is clicked, provide an ON pulse for input.getBool(31)
        -- simulator:setInputNumber(31, simulator:getSlider(1))        -- set input 31 to the value of slider 1

        -- simulator:setInputBool(32, simulator:getIsToggled(2))       -- make button 2 a toggle, for input.getBool(32)
        -- simulator:setInputNumber(32, simulator:getSlider(2) * 50)   -- set input 32 to the value from slider 2 * 50
    end;
end
---@endsection


--[====[ IN-GAME CODE ]====]

-- try require("Folder.Filename") to include code from another file in this, so you can store code in libraries
-- the "LifeBoatAPI" is included by default in /_build/libs/ - you can use require("LifeBoatAPI") to get this, and use all the LifeBoatAPI.<functions>!

local Nouns = {
    RESET = 0,
    IDLE = 1,
    SET_MAIN_MENU = 2,
    MAIN_MENU_P2 = 3,
    SET_S_POS = 4,
    SET_T_POS = 5,
    SET_T_VEL = 6,
    SET_CFG = 7,
    S_POS_CPOS = 8,
    S_POS_APOS = 9,
    S_POS_CUSTOM = 10,
    S_POS_PRED = 11,
    T_POS_REAL = 12,
    T_POS_ASTRO = 13,
    T_POS_ACC = 14,
    T_VEL_ZERO = 15,
    T_VEL_CUSTOM = 16,
    T_VEL_OGAHD = 17
}

Noun = Nouns.RESET

-- Input Value stuff
Input_Active = false
Input_Remaining = 0
Input_Active_Field = 0

Input_Values = {}
Input_Descs = {"VRTTES"}

COLOR_LETTER = {0, 255, 0}
COLOR_BACKGROUND = {0, 0, 0}
COLOR_SELECTED_BACKGROUND = {0, 255, 255}

Input_Finished = false

Current_Entry = ""

-- Menu Stuff
Menu_Active = false
Menu_Text = ""
Menu_Elements = {}
Menu_Target_Noun = {}
Menu_Status = {}

Menu_Element_Pressed = false

Menu_Selection = 0

-- Status Stuff
Start_Pos_Mode = 0
Start_X = 0
Start_Y = 0
Start_Z = 0

Target_Mode = 0 -- Not really required to track mode. But whatever, we need to track if the target is set anyway.
Target_X = 0
Target_Y = 0
Target_Z = 0


function run_button(char_string, x, y, is_press, s_x, s_y, color_letter, color_bg, color_bg_select)

    screen.setColor(255, 255, 255)
    screen.drawRect(x, y, 5, 6)

    local is_select = is_press and s_x >= (x + 1) and s_x <= (x + 4) and s_y >= (y + 1) and s_y <= (y + 5)

    screen.setColor(table.unpack(color_bg))

    if is_select then
        screen.setColor(table.unpack(color_bg_select))
    end

    screen.drawRectF(x + 1, y + 1, 4, 5)

    screen.setColor(table.unpack(color_letter))

    screen.drawText(x + 1, y + 1, char_string)

    return is_select
end

function run_menu_button(name, current_status, y, is_touch, touch_y)

    is_touch = is_touch and touch_y > y and touch_y < y + 6

    if type(current_status) == "boolean" then
        current_status = current_status and 1 or 0
    end

    screen.setColor(255, 255, 255)

    if is_touch then
        screen.setColor(0, 255, 255)
    end

    screen.drawRectF(0, y, 32, 7)

    screen.setColor(100, 100, 100)
    screen.drawRect(0, y, 31, 6)

    screen.setColor(0, 0, 0)
    screen.drawText(7, y + 1, name)

    screen.setColor(255, 0, 0)
    if current_status == 1 then
        screen.setColor(0, 255, 0)
    elseif current_status == 2 then
        screen.setColor(0, 0, 255)
    end

    screen.drawCircleF(4, y + 3.5, 2.5)

    return is_touch
end

function start_menu(name, items, after_nouns, status)
    Menu_Elements = items
    Menu_Target_Noun = after_nouns
    Menu_Status = status
    Menu_Text = name

    Noun = Nouns.IDLE
    Menu_Selection = 0
    Menu_Active = true
end

function start_input(descriptions)
    local num_inputs = #descriptions

    Input_Active = true
    Input_Remaining = num_inputs
    Input_Active_Field = 1

    Input_Values = {}
    Input_Descs = descriptions

    Input_Finished = false
end

touch_x = 0
touch_y = 0
pressed = false

function onTick()
    touch_x = input.getNumber(3)
    touch_y = input.getNumber(4)
    pressed = input.getBool(1)
end

function onDraw()

    press_reset = run_button("R", 26, 25, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)

    if press_reset then
        Noun = Nouns.RESET

        Input_Active = false
        Menu_Active = false

        screen.setColor(255, 255, 255)
        screen.drawText(4, 13, "RESET")

        return
    end

    if Input_Active then
        
        press_1 = run_button("1", 0, 7, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_2 = run_button("2", 5, 7, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_3 = run_button("3", 10, 7, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_4 = run_button("4", 0, 13, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_5 = run_button("5", 5, 13, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_6 = run_button("6", 10, 13, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_7 = run_button("7", 0, 19, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_8 = run_button("8", 5, 19, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_9 = run_button("9", 10, 19, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_0 = run_button("0", 0, 25, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_d = run_button(".", 5, 25, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_n = run_button("-", 10, 25, pressed, touch_x, touch_y, COLOR_LETTER, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_del = run_button("X", 15, 19, pressed, touch_x, touch_y, {255, 0, 0}, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)
        press_e = run_button("E", 15, 25, pressed, touch_x, touch_y, {0, 255, 0}, COLOR_BACKGROUND, COLOR_SELECTED_BACKGROUND)

        local digit_presses = {
            [press_0] = "0",
            [press_1] = "1",
            [press_2] = "2",
            [press_3] = "3",
            [press_4] = "4",
            [press_5] = "5",
            [press_6] = "6",
            [press_7] = "7",
            [press_8] = "8",
            [press_9] = "9",
        }

        local any_press =
            press_0 or press_1 or press_2 or press_3 or press_4 or
            press_5 or press_6 or press_7 or press_8 or press_9 or
            press_d or press_n or press_e or press_del

        if any_press then
            if not kp_reg then
                for pressed, value in pairs(digit_presses) do
                    if pressed then
                        Current_Entry = Current_Entry .. value
                    end
                end

                if press_d and not Current_Entry:find("%.") then
                    Current_Entry = Current_Entry .. "."
                end

                if press_n and #Current_Entry == 0 then
                    Current_Entry = "-"
                end

                if press_del then
                    Current_Entry = Current_Entry:sub(1, -2)
                end

                if press_e then
                    Input_Values[Input_Active_Field] = tonumber(Current_Entry)
                    Current_Entry = ""
                    Input_Active_Field = Input_Active_Field + 1
                    Input_Remaining = Input_Remaining - 1

                    if Input_Remaining == 0 then
                        Input_Finished = true
                        Input_Active = false
                        return
                    end
                end

                kp_reg = true
            end
        else
            kp_reg = false
        end

        screen.setColor(255, 0, 255)
        screen.drawRect(0, 0, 31, 6)

        screen.setColor(255, 255, 255)

        local bt_pos = math.min(1, 32 - 5 * #Current_Entry)

        screen.drawText(bt_pos, 1, Current_Entry)

        screen.setColor(0, 255, 255)

        screen.drawText(17, 7, string.sub(Input_Descs[Input_Active_Field], 1, 3))
        screen.drawText(17, 13, string.sub(Input_Descs[Input_Active_Field], 4, 6))

        return
    end

    if Menu_Active then
        screen.setColor(255, 255, 255)
        screen.drawText(13 - 2.5 * #Menu_Text, 26, Menu_Text)

        screen.setColor(100, 100, 100)

        element_pressed = 0
        
        for i = 1, #Menu_Elements do
            b_press = run_menu_button(Menu_Elements[i], Menu_Status[i], 6 * i - 6, pressed, touch_y)

            if b_press then
                element_pressed = i
            end
        end

        if element_pressed ~= 0 then
            Menu_Selection = element_pressed;
        end

        if Menu_Selection ~= 0 and element_pressed == 0 then
            -- Press released. Signal the next thing. 
            Noun = Menu_Target_Noun[Menu_Selection]
            Menu_Active = false
            print(Noun)
        end
    end

    if Noun == Nouns.RESET then
        Noun = Nouns.SET_MAIN_MENU
        return
    end

    if Noun == Nouns.SET_MAIN_MENU then
        start_menu("NAV 1", {"S-POS", "T-POS", "T-VEL", "PAGE2"}, {Nouns.SET_S_POS, Nouns.SET_T_POS, Nouns.SET_T_VEL, Nouns.MAIN_MENU_P2}, {Start_Pos_Mode ~= 0, Target_Mode ~= 0, 0, 2})
        return
    end

    if Noun == Nouns.MAIN_MENU_P2 then
        start_menu("NAV 2", {"PAGE1", "CFG", "LNCH"}, {Nouns.SET_MAIN_MENU, Nouns.SET_CFG, Nouns.IDLE}, {2, 1, 0})
        return
    end

    if Noun == Nouns.SET_S_POS then
        start_menu("S-POS", {"C-POS", "A-POS", "PRED", "CUSTM"}, {Nouns.S_POS_CPOS, Nouns.S_POS_APOS, Nouns.S_POS_PRED, Nouns.S_POS_CUSTOM}, {2, 2, 2, 2})
        return
    end

    if Noun == Nouns.S_POS_CPOS then
        -- TODO: Save CPOS here
        Start_Pos_Mode = 1
        Noun = Nouns.SET_MAIN_MENU

        return
    end

    if Noun == Nouns.S_POS_APOS then
        if Input_Finished then
            -- Unpack from register.
            Start_Pos_Mode = 2
            print("SET")
            print(Start_Pos_Mode)
            Start_X = 0
            Start_Y = Input_Values[1]
            Start_Z = 0
            Noun = Nouns.SET_MAIN_MENU
            Input_Finished = false
            return
        end

        if not Input_Active then
            -- Setup the input.
            start_input({"LCHALT"})
        end

        return
    end

    if Noun == Nouns.S_POS_CUSTOM then
        if Input_Finished then
            -- Unpack from register.
            Start_Pos_Mode = 3
            Start_X = Input_Values[1]
            Start_Y = Input_Values[2]
            Start_Z = Input_Values[3]
            Noun = Nouns.SET_MAIN_MENU
            Input_Finished = false
            return
        end

        if not Input_Active then
            -- Setup the input.
            start_input({"POS X ", "POS Y ", "POS Z "})
        end
    end

    if Noun == Nouns.S_POS_PRED then
        if Input_Finished then
            -- Unpack from register
            Start_Pos_Mode = 4
            Start_X = Input_Values[1]
            Start_Y = 0
            Start_Z = 0
            Noun = Nouns.SET_MAIN_MENU
            Input_Finished = false
            return
        end

        if not Input_Active then
            -- Setup the input
            start_input({"PP TME"})
        end
    end

    if Noun == Nouns.SET_T_POS then
        start_menu("T-POS", {"REAL", "ASTRO", "ACC"}, {Nouns.T_POS_REAL, Nouns.T_POS_ASTRO, Nouns.T_POS_ACC}, {2, 2, 2})
        return
    end

    if Noun == Nouns.T_POS_REAL then
        print("CK0", Input_Finished)
        if Input_Finished then
            -- Unpack from register.
            Target_Mode = 1
            Target_X = Input_Values[1]
            Target_Y = Input_Values[2]
            Target_Z = Input_Values[3]
            Noun = Nouns.SET_MAIN_MENU
            Input_Finished = false
            return
        end

        if not Input_Active then
            -- Setup the input.
            start_input({"POS X ", "POS Y ", "POS Z "})
        end
    end

    if Noun == Nouns.T_POS_ASTRO then
        if Input_Finished then
            -- Unpack from register.
            Target_Mode = 2
            Target_X = Input_Values[1]
            Target_Y = Input_Values[2]
            Target_Z = Input_Values[3]

            -- TODO: Do the astro -> real conversion here.
            Noun = Nouns.SET_MAIN_MENU
            Input_Finished = false
            return
        end

        if not Input_Active then
            -- Setup the input.
            start_input({"POS X ", "POS Y ", "POS Z "})
        end
    end

    if Noun == Nouns.T_POS_ACC then
        -- TODO: Figure out how this mode is gonna work
        Target_Mode = 3

        Noun = Nouns.SET_MAIN_MENU
    end




end



