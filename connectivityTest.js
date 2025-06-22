
const axios = require('axios');

async function testConnectivity(url) {
    try {
        const response = await axios.get(url);
        if (response.status === 200) {
            console.log('Connectivity test passed.');
        } else {
            console.log('Connectivity test failed with status:', response.status);
        }
    } catch (error) {
        console.log('Connectivity test failed with error:', error.message);
    }
}

// Example usage
testConnectivity('https://example.com');
