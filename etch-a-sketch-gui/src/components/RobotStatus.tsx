import React, { useState, useEffect } from "react";
import { Box, Heading, Text, HStack, Button, useToast } from "@chakra-ui/react";
import axios from "axios";

interface RobotStatusProps {
    etchbotName: string;
}

interface RobotStatusData {
    state: string;
    current_drawing: string | null;
    camera_connected: boolean;
    camera_recording: boolean;
    recording_mode: boolean;  // Reflects if recording while drawing is enabled
    paused: boolean;
    cooldown_remaining: number;
}

const API_URL = import.meta.env.VITE_API_URL || 'http://localhost:5010';

const RobotStatus: React.FC<RobotStatusProps> = ({ etchbotName }) => {
    const [status, setStatus] = useState<RobotStatusData | null>(null);
    // const [loading, setLoading] = useState(true);
    const [error, setError] = useState<string | null>(null);
    // const [updating, setUpdating] = useState(false);
    const toast = useToast();

    const fetchStatus = async () => {
        // setUpdating(true);
        try {
            const response = await axios.get(`${API_URL}/etchbot/${etchbotName}/status`);
            const data = response.data as RobotStatusData;
            setStatus(data);
            setError(null);
        } catch (err) {
            console.error("Failed to fetch robot status", err);
            setError("Failed to retrieve robot status.");
        } finally {
            // setLoading(false);
            // setTimeout(() => setUpdating(false), 500); // Stop spinning after 1 second
        }
    };

    useEffect(() => {
        fetchStatus(); // Initial fetch

        const intervalId = setInterval(fetchStatus, 1000); // Refresh every second

        return () => clearInterval(intervalId); // Cleanup the interval on component unmount
    }, [etchbotName]);

    const handlePauseResume = async () => {
        try {
            if (status?.paused) {
                // If the robot is paused, resume it
                await axios.post(`${API_URL}/etchbot/${etchbotName}/resume`);
                toast({
                    title: "Robot resumed.",
                    status: "success",
                    duration: 2000,
                    isClosable: true,
                });
            } else {
                // If the robot is running, pause it
                await axios.post(`${API_URL}/etchbot/${etchbotName}/pause`);
                toast({
                    title: "Robot paused.",
                    status: "success",
                    duration: 2000,
                    isClosable: true,
                });
            }
        } catch (err) {
            console.error("Failed to pause/resume robot", err);
            toast({
                title: "Failed to pause/resume robot.",
                status: "error",
                duration: 2000,
                isClosable: true,
            });
        }
    };

    const handleToggleRecording = async () => {
        try {
            const recordingMode = !status?.recording_mode;
            await axios.post(`${API_URL}/etchbot/${etchbotName}/set_recording_mode`, {
                recording_mode: recordingMode,
            });
            toast({
                title: `Recording while drawing ${recordingMode ? "enabled" : "disabled"}.`,
                status: "success",
                duration: 2000,
                isClosable: true,
            });
        } catch (err) {
            console.error("Failed to toggle recording mode", err);
            toast({
                title: "Failed to toggle recording mode.",
                status: "error",
                duration: 2000,
                isClosable: true,
            });
        }
    };

    const handleClearError = async () => {
        try {
            await axios.post(`${API_URL}/etchbot/${etchbotName}/clear_error`);
            toast({
                title: "Error cleared and EtchBot reset.",
                status: "success",
                duration: 2000,
                isClosable: true,
            });
        } catch (err) {
            console.error("Failed to clear error", err);
            toast({
                title: "Failed to clear error.",
                status: "error",
                duration: 2000,
                isClosable: true,
            });
        }
    };

    // if (loading) {
    //     return <Spinner />;
    // }

    if (error) {
        return <Text color="red.500">{error}</Text>;
    }

    return (
        <Box>
            <HStack mb={3}>
                <Heading as="h3" size="md">
                    Robot Status
                </Heading>
                {/* {updating && <Spinner size="sm" />} Show spinner when updating */}
            </HStack>
            {status ? (
                <Box>
                    <Text><strong>State:</strong> {status.state}</Text>
                    <Text><strong>Current Drawing:</strong> {status.current_drawing || "None"}</Text>
                    <Text><strong>Camera Connected:</strong> {status.camera_connected ? "Yes" : "No"}</Text>
                    <Text><strong>Camera Recording:</strong> {status.camera_recording ? "Yes" : "No"}</Text>
                    <Text><strong>Recording Enabled:</strong> {status.recording_mode ? "Yes" : "No"}</Text>
                    <Text><strong>Paused:</strong> {status.paused ? "Yes" : "No"}</Text>
                    <Text><strong>Cooldown Remaining:</strong> {status.cooldown_remaining.toFixed(1)} seconds</Text> {/* New field */}

                    {/* Pause/Resume Button */}
                    <Button
                        colorScheme={status.paused ? "green" : "yellow"}
                        onClick={handlePauseResume}
                        mt={4}
                    >
                        {status.paused ? "Resume" : "Pause"}
                    </Button>

                    {/* Enable/Disable Recording While Drawing */}
                    <Button
                        colorScheme="blue"
                        onClick={handleToggleRecording}
                        mt={4}
                        ml={4}
                    >
                        {status.recording_mode ? "Disable Recording While Drawing" : "Enable Recording While Drawing"}
                    </Button>

                    {/* Clear Error & Reset */}
                    <Button
                        colorScheme="green"
                        onClick={handleClearError}
                        mt={4}
                        ml={4}
                        isDisabled={status.state !== "ERROR"} // Disable the button if the state is not "ERROR"
                        opacity={status.state === "ERROR" ? 1 : 0.5} // Change opacity to indicate it's disabled
                    >
                        Clear Error & Reset
                    </Button>
                </Box>
            ) : (
                <Text>No status available.</Text>
            )}
        </Box>
    );
};

export default RobotStatus;
