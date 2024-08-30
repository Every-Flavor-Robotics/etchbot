import React, { useState, useEffect } from "react";
import { ChakraProvider, Box, Heading, Flex, Text, Spinner, HStack, Circle } from "@chakra-ui/react";
import axios from "axios";
import RobotPanel from "./components/RobotPanel"; // Import the new RobotPanel component

interface Etchbot {
    name: string;
    state?: string;
}

const stateColors: { [key: string]: string } = {
    DISCONNECTED: "gray",
    READY: "green",
    DRAWING: "yellow",
    DRAWING_COMPLETE: "green",
    ERASING: "yellow",
    ERASING_COMPLETE: "green",
    ERROR: "red",
};

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:5010';


const App: React.FC = () => {
    const [etchbots, setEtchbots] = useState<Etchbot[]>([]);
    const [selectedEtchbot, setSelectedEtchbot] = useState<Etchbot | null>(null);
    const [loading, setLoading] = useState(true);

    // Fetch etchbots and their states
    useEffect(() => {
        const fetchEtchbotsAndStates = async () => {
            try {
                const response = await axios.get("${API_URL}/etchbots");
                const etchbotsData = response.data.etchbots;

                if (Array.isArray(etchbotsData)) {
                    const etchbotsWithState = await Promise.all(
                        etchbotsData.map(async (name) => {
                            const stateResponse = await axios.get("${API_URL}/etchbot/state", {
                                params: { name }, // Pass the name as a query parameter
                            });
                            return {
                                name,
                                state: stateResponse.data.state,
                            };
                        })
                    );
                    setEtchbots(etchbotsWithState);
                    setSelectedEtchbot((prevSelected) =>
                        etchbotsWithState.find((bot) => bot.name === prevSelected?.name) || etchbotsWithState[0] || null
                    );
                } else {
                    console.error("Unexpected response format:", response.data);
                }
            } catch (err) {
                console.error("Failed to fetch etchbots and their states", err);
            } finally {
                setLoading(false);
            }
        };

        fetchEtchbotsAndStates(); // Initial fetch

        const etchbotsInterval = setInterval(fetchEtchbotsAndStates, 5000); // Poll every 5 seconds

        return () => clearInterval(etchbotsInterval); // Cleanup interval on component unmount
    }, []);

    const handleEtchbotSelect = (etchbot: Etchbot) => {
        setSelectedEtchbot(etchbot);
    };

    return (
        <ChakraProvider>
            <Flex>
                {/* Sidebar */}
                <Box
                    w="250px"
                    p={5}
                    bg="gray.100"
                    height="100vh"  // Extend the sidebar to the full viewport height
                    display="flex"
                    flexDirection="column"
                >
                    <Heading as="h2" size="md" mb={4}>
                        Etchbots
                    </Heading>
                    {loading ? (
                        <Spinner />
                    ) : (
                        etchbots.length > 0 ? (
                            etchbots.map((etchbot) => (
                                <HStack
                                    key={etchbot.name}
                                    p={2}
                                    mb={2}
                                    cursor="pointer"
                                    bg={selectedEtchbot?.name === etchbot.name ? "teal.100" : "white"}
                                    onClick={() => handleEtchbotSelect(etchbot)}
                                >
                                    <Circle size="10px" bg={stateColors[etchbot.state || "DISCONNECTED"]} />
                                    <Text>{etchbot.name}</Text>
                                </HStack>
                            ))
                        ) : (
                            <Text>No etchbots found.</Text>
                        )
                    )}
                </Box>

                {/* Main Content */}
                <Box flex="1" p={5}>
                    {selectedEtchbot ? (
                        <RobotPanel etchbotName={selectedEtchbot.name} />
                    ) : (
                        <Text>Select an etchbot to view the panel.</Text>
                    )}
                </Box>
            </Flex>
        </ChakraProvider>
    );
};

export default App;
