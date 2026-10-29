sim = require 'sim'
simVision = require 'simVision'

local lidarHandle = nil
local sequence = 0
local propertyName = 'signal.faizal_lidar_packet_v2'

function sysCall_init()
    local sensors = {}

    for i = 1, 4 do
        sensors[i] = sim.getObject('../sensor', {index = i - 1})
    end

    local pointCloud = sim.getObject('../ptCloud')

    lidarHandle = simVision.createVelodyneVPL16(
        sensors,
        5,          -- Frekuensi putaran: 5 Hz
        2 + 8,      -- Cartesian, titik terbaru, emissive
        2,          -- Ukuran titik pada tampilan
        {1, 4},     -- Pewarnaan berdasarkan jarak
        0.999,      -- Penyesuaian visual titik
        pointCloud
    )

    sequence = 0

    local initialPacket = sim.packFloatTable({0, 0, 0})

    sim.setBufferProperty(
        sim.handle_scene,
        propertyName,
        initialPacket,
        {}
    )
end

function sysCall_sensing()

    local raw = simVision.handleVelodyneVPL16(
        lidarHandle
            + sim.handleflag_abscoords
            + sim.handleflag_codedstring,
        sim.getSimulationTimeStep()
    )

    local xyz = {}

    if raw ~= nil then
        if type(raw) == 'table' then
            xyz = raw
        elseif #raw > 0 then
            xyz = sim.unpackFloatTable(raw)
        end
    end

    if #xyz % 3 ~= 0 then
        error('Data LiDAR tidak sesuai format XYZ: jumlah angka bukan kelipatan 3.')
    end

    sequence = sequence + 1
    local numberOfPoints = #xyz / 3

    local packet = {
        sequence,
        sim.getSimulationTime(),
        numberOfPoints
    }

    for i = 1, #xyz do
        packet[#packet + 1] = xyz[i]
    end

    sim.setBufferProperty(
        sim.handle_scene,
        propertyName,
        sim.packFloatTable(packet),
        {}
    )
end

function sysCall_cleanup()
    if lidarHandle ~= nil then
        simVision.destroyVelodyneVPL16(lidarHandle)
        lidarHandle = nil
    end

end