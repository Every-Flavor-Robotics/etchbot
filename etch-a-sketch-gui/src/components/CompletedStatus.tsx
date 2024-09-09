import React, { useState, useEffect } from "react";
import { Box, Heading, Table, Thead, Tbody, Tr, Th, Td, Spinner, Text, HStack, IconButton } from "@chakra-ui/react";
import { FaDownload } from "react-icons/fa"; // Import the download icon
import axios from "axios";

interface CompletedItem {
    name: string;
    length: number;
    ready: boolean;
}

interface CompletedStatusProps {
    etchbotName: string;
}

// const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:5010';
const API_URL = 'http://localhost:5010';


const CompletedStatus: React.FC<CompletedStatusProps> = ({ etchbotName }) => {
    const [completed, setCompleted] = useState<CompletedItem[]>([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState<string | null>(null);
    const [updating, setUpdating] = useState(false);

    const fetchCompletedStatus = async () => {
        setUpdating(true);
        try {
            const response = await axios.get(API_URL + "/etchbot/completed", {
                params: { name: etchbotName },
            });
            setCompleted(Object.values(response.data.completed));
            setError(null);
        } catch (err) {
            console.error("Failed to fetch completed drawings", err);
            setError("Failed to retrieve completed drawings information.");
        } finally {
            setLoading(false);
            setTimeout(() => setUpdating(false), 500); // Stop spinning after 1 second
        }
    };

    const handleDownload = async (name: string, index: number) => {
        try {
            const response = await axios.post(API_URL + `/etchbot/${etchbotName}/download_zip`, {
                drawing_index: index, // Pass the index instead of the name
            }, {
                responseType: 'blob', // Important to download files
            });

            const url = window.URL.createObjectURL(new Blob([response.data]));
            const link = document.createElement('a');
            link.href = url;
            link.setAttribute('download', `${name}.zip`); // Filename for download
            document.body.appendChild(link);
            link.click();
            link.remove();
        } catch (err) {
            console.error("Failed to download artifact", err);
            setError("Failed to download artifact.");
        }
    };

    useEffect(() => {
        fetchCompletedStatus(); // Initial fetch

        const intervalId = setInterval(fetchCompletedStatus, 5000); // Refresh every 5 seconds

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
                    Completed Drawings
                </Heading>
                {updating && <Spinner size="sm" />} {/* Show spinner when updating */}
            </HStack>
            {completed.length > 0 ? (
                <Table variant="striped" colorScheme="gray" size="md" borderWidth="1px" borderRadius="lg" overflow="hidden">
                    <Thead>
                        <Tr>
                            <Th>Drawing Name</Th>
                            <Th>Number of Gcodes</Th>
                            <Th>Download</Th> {/* Replace Status with Download */}
                        </Tr>
                    </Thead>
                    <Tbody>
                        {completed.map((item, index) => (
                            <Tr key={item.name}>
                                <Td>{item.name}</Td>
                                <Td>{item.length}</Td>
                                <Td>
                                    <IconButton
                                        icon={<FaDownload />}
                                        aria-label="Download"
                                        onClick={() => handleDownload(item.name, index)} // Pass the index here
                                    />
                                </Td>
                            </Tr>
                        ))}
                    </Tbody>
                </Table>
            ) : (
                <Text>No completed drawings.</Text>
            )}
        </Box>
    );
};

export default CompletedStatus;
