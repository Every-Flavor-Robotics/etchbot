import React, { useState, useEffect } from "react";
import { Box, Heading, Text, Spinner, HStack } from "@chakra-ui/react";
import axios from "axios";

interface RobotStatusProps {
    etchbotName: string;
}

interface RobotStatusData {
    state: string;
    current_drawing: string | null;
    camera_connected: boolean;
    camera_recording: boolean;
}

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:5010';


const RobotStatus: React.FC<RobotStatusProps> = ({ etchbotName }) => {
    const [status, setStatus] = useState<RobotStatusData | null>(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState<string | null>(null);
    const [updating, setUpdating] = useState(false);

    const fetchStatus = async () => {
        setUpdating(true);
        try {
            const response = await axios.get(`${API_URL}/etchbot/${etchbotName}/status`);
            setStatus(response.data);
            setError(null);
        } catch (err) {
            console.error("Failed to fetch robot status", err);
            setError("Failed to retrieve robot status.");
        } finally {
            setLoading(false);
            setTimeout(() => setUpdating(false), 500); // Stop spinning after 1 second
        }
    };

    useEffect(() => {
        fetchStatus(); // Initial fetch

        const intervalId = setInterval(fetchStatus, 1000); // Refresh every 5 seconds

        return () => clearInterval(intervalId); // Cleanup the interval on component unmount
    }, [etchbotName]);

    if (loading) {
        return <Spinner />;
    }

    if (error) {
        return <Text color="red.500">{error}</Text>;
    }

    return (
        <Box>
            <HStack mb={3}>
                <Heading as="h3" size="md">
                    Robot Status
                </Heading>
                {updating && <Spinner size="sm" />} {/* Show spinner when updating */}
            </HStack>
            {status ? (
                <Box>
                    <Text><strong>State:</strong> {status.state}</Text>
                    <Text><strong>Current Drawing:</strong> {status.current_drawing || "None"}</Text>
                    <Text><strong>Camera Connected:</strong> {status.camera_connected ? "Yes" : "No"}</Text>
                    <Text><strong>Camera Recording:</strong> {status.camera_recording ? "Yes" : "No"}</Text>
                </Box>
            ) : (
                <Text>No status available.</Text>
            )}
        </Box>
    );
};

export default RobotStatus;
