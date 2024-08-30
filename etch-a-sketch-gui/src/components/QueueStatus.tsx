import React, { useState, useEffect } from "react";
import { Box, Heading, Table, Thead, Tbody, Tr, Th, Td, Spinner, Text, HStack } from "@chakra-ui/react";
import axios from "axios";

interface QueueItem {
    name: string;
    length: number;
    ready: boolean;
}

interface QueueStatusProps {
    etchbotName: string;
}

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:5010';


const QueueStatus: React.FC<QueueStatusProps> = ({ etchbotName }) => {
    const [queue, setQueue] = useState<QueueItem[]>([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState<string | null>(null);
    const [updating, setUpdating] = useState(false);

    const fetchQueueStatus = async () => {
        setUpdating(true);
        try {
            const response = await axios.get("${API_URL}/etchbot/queue", {
                params: { name: etchbotName },
            });
            setQueue(Object.values(response.data.queue));
            setError(null);
        } catch (err) {
            console.error("Failed to fetch queue status", err);
            setError("Failed to retrieve queue information.");
        } finally {
            setLoading(false);
            setTimeout(() => setUpdating(false), 500); // Stop spinning after 1 second
        }
    };

    useEffect(() => {
        fetchQueueStatus(); // Initial fetch

        const intervalId = setInterval(fetchQueueStatus, 5000); // Refresh every 5 seconds

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
                    Queue Status
                </Heading>
                {updating && <Spinner size="sm" />} {/* Show spinner when updating */}
            </HStack>
            {queue.length > 0 ? (
                <Table variant="striped" colorScheme="gray" size="md" borderWidth="1px" borderRadius="lg" overflow="hidden">
                    <Thead>
                        <Tr>
                            <Th>Drawing Name</Th>
                            <Th>Number of Gcodes</Th>
                            <Th>Status</Th>
                        </Tr>
                    </Thead>
                    <Tbody>
                        {queue.map((item) => (
                            <Tr key={item.name}>
                                <Td>{item.name}</Td>
                                <Td>{item.length}</Td>
                                <Td>{item.ready ? "Ready" : "Processing"}</Td>
                            </Tr>
                        ))}
                    </Tbody>
                </Table>
            ) : (
                <Text>No items in the queue.</Text>
            )}
        </Box>
    );
};

export default QueueStatus;
