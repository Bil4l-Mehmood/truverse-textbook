import { signUp } from './simple-auth.js';

(async () => {
  try {
    const result = await signUp('bilal@pctsc.com', 'Ahmed2008!', 'Bilal Ahmed');
    console.log('✅ Test user created:', result.user);
    console.log('✅ Token:', result.session.token);
    process.exit(0);
  } catch (error) {
    console.error('❌ Error:', error.message);
    process.exit(1);
  }
})();
