const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;
const migrationManifest = require('./migration/manifest.json');

const normalizeRoute = (route) => {
  if (route === '/') {
    return route;
  }
  return `/${route.replace(/^\/|\/$/g, '')}/`;
};

const legacyPageByRoute = new Map(
  migrationManifest.pages.map((page) => [
    normalizeRoute(page.route),
    page.source.replace('.', '/'),
  ]),
);

async function createConfig() {
  const remarkMath = (await import('remark-math')).default;
  const rehypeKatex = (await import('rehype-katex')).default;

  return {
    title: 'jMPC Toolbox',
    tagline: 'A free MATLAB toolbox for fast Model Predictive Control',
    url: process.env.SITE_URL || 'https://jonathancurrie.github.io',
    baseUrl: process.env.BASE_URL || '/jMPC/',
    organizationName: 'jonathancurrie',
    projectName: 'jMPC',
    trailingSlash: true,
    onBrokenLinks: 'throw',
    onBrokenAnchors: 'throw',
    onDuplicateRoutes: 'throw',

    markdown: {
      format: 'detect',
    },

    presets: [
      [
        'classic',
        {
          docs: {
            routeBasePath: '/',
            sidebarPath: require.resolve('./sidebars.js'),
            breadcrumbs: true,
            remarkPlugins: [remarkMath],
            rehypePlugins: [rehypeKatex],
          },
          blog: false,
          pages: false,
          theme: {
            customCss: require.resolve('./src/css/custom.css'),
          },
          gtag: {
            trackingID: 'G-9GBVJ653GH',
          },
          sitemap: {
            changefreq: 'monthly',
            priority: 0.5,
          },
        },
      ],
    ],

    plugins: [
      [
        '@docusaurus/plugin-client-redirects',
        {
          createRedirects(existingPath) {
            const legacyPage = legacyPageByRoute.get(normalizeRoute(existingPath));
            return legacyPage ? [`/pmwiki.php/${legacyPage}`] : undefined;
          },
        },
      ],
      [
        require.resolve('@cmfcmf/docusaurus-search-local'),
        {
          indexDocs: true,
          indexBlog: false,
          indexPages: false,
          language: 'en',
          maxSearchResults: 8,
        },
      ],
    ],

    themeConfig: {
      metadata: [
        {
          name: 'description',
          content:
            'Documentation for jMPC, a MATLAB toolbox for fast linear Model Predictive Control.',
        },
      ],
      colorMode: {
        defaultMode: 'light',
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'jMPC Toolbox',
        hideOnScroll: true,
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docsSidebar',
            position: 'left',
            label: 'Documentation',
          },
          {
            to: '/examples/',
            label: 'Examples',
            position: 'left',
          },
          {
            href: 'https://github.com/jonathancurrie/jMPC',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'jMPC',
            items: [
              {label: 'Getting started', to: '/examples/'},
              {label: 'Download', to: '/download/'},
              {label: 'License', to: '/license/'},
            ],
          },
          {
            title: 'Control Engineering',
            items: [
              {
                label: 'Main website',
                href: 'https://www.controlengineering.co.nz/',
              },
              {
                label: 'jMPC on GitHub',
                href: 'https://github.com/jonathancurrie/jMPC',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Control Engineering.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['matlab'],
      },
    },
  };
}

module.exports = createConfig;
