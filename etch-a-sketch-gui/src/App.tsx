import React, { useState, useEffect } from "react";
import { ChakraProvider, Box, Flex, Text, Spinner } from "@chakra-ui/react";
import axios from "axios";
import RobotPanel from "./components/RobotPanel";
import { colors, stateColors } from "./theme";

const API_URL = import.meta.env.VITE_API_URL || "http://localhost:5010";

interface Etchbot { name: string; state?: string; }
interface EtchbotsResponse { etchbots: string[]; }
interface EtchbotStateResponse { state: string; }

const App: React.FC = () => {
  const [etchbots, setEtchbots] = useState<Etchbot[]>([]);
  const [selected, setSelected] = useState<Etchbot | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchAll = async () => {
      try {
        const res = await axios.get<EtchbotsResponse>(`${API_URL}/etchbots`);
        if (!Array.isArray(res.data.etchbots)) return;
        const bots = await Promise.all(
          res.data.etchbots.map(async (name) => {
            const s = await axios.get<EtchbotStateResponse>(`${API_URL}/etchbot/state`, { params: { name } });
            return { name, state: s.data.state };
          })
        );
        setEtchbots(bots);
        setSelected(prev => bots.find(b => b.name === prev?.name) || bots[0] || null);
      } catch (e) {
        console.error("Failed to fetch etchbots", e);
      } finally {
        setLoading(false);
      }
    };
    fetchAll();
    const id = setInterval(fetchAll, 5000);
    return () => clearInterval(id);
  }, []);

  return (
    <ChakraProvider>
      <Box minH="100vh" bg={colors.surface} display="flex" flexDirection="column">
        {/* Top bar */}
        <Flex
          align="center"
          gap="10px"
          px="20px"
          py="12px"
          bg={colors.white}
          borderBottom="3px solid #f0f0f0"
          flexShrink={0}
        >
          {/* Logo */}
          <Flex gap="4px" align="center">
            <Box w="26px" h="26px" bg={colors.blue} borderRadius="8px" display="flex" alignItems="center" justifyContent="center">
              <Text color="white" fontSize="13px" fontWeight="900">E</Text>
            </Box>
            <Box w="8px" h="26px" bg={colors.amber} borderRadius="4px" />
            <Box w="8px" h="26px" bg={colors.coral} borderRadius="4px" />
          </Flex>
          <Text fontWeight="800" fontSize="15px" color={colors.ink} letterSpacing="-0.3px">etchbot</Text>

          {/* Robot tabs */}
          <Flex ml="16px" gap="6px">
            {loading ? (
              <Spinner size="sm" />
            ) : etchbots.map(bot => (
              <Box
                key={bot.name}
                bg={selected?.name === bot.name ? colors.blue : "#f0f0f5"}
                color={selected?.name === bot.name ? "white" : "#888"}
                fontSize="10px"
                fontWeight="700"
                px="12px"
                py="4px"
                borderRadius="20px"
                cursor="pointer"
                onClick={() => setSelected(bot)}
                display="flex"
                alignItems="center"
                gap="5px"
              >
                <Box
                  w="7px"
                  h="7px"
                  borderRadius="50%"
                  bg={stateColors[bot.state || "DISCONNECTED"]}
                  flexShrink={0}
                />
                {bot.name}
              </Box>
            ))}
          </Flex>

          {/* Status badge */}
          {selected && (
            <Box
              ml="auto"
              display="flex"
              alignItems="center"
              gap="6px"
              bg="#e8f9f0"
              borderRadius="20px"
              px="12px"
              py="4px"
              border="1.5px solid #b6ebd0"
            >
              <Box w="8px" h="8px" borderRadius="50%" bg={stateColors[selected.state || "DISCONNECTED"]} />
              <Text color="#16a34a" fontSize="10px" fontWeight="700">
                {selected.state || "Disconnected"}
              </Text>
            </Box>
          )}
        </Flex>

        {/* Main content */}
        <Box flex={1} overflow="auto">
          {selected ? (
            <RobotPanel etchbotName={selected.name} />
          ) : (
            <Flex h="100%" align="center" justify="center">
              <Text color="#888">No robots found.</Text>
            </Flex>
          )}
        </Box>
      </Box>
    </ChakraProvider>
  );
};

export default App;
